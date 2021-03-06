

#include "madara/knowledge/KnowledgeBase.h"
#include "madara/threads/Threader.h"
#include "gams/controllers/BaseController.h"
#include "gams/loggers/GlobalLogger.h"

// DO NOT DELETE THIS SECTION

// begin algorithm includes
// end algorithm includes

// begin platform includes
#include "platforms/boat.h"
// end platform includes

// begin thread includes
#include "threads/localization.h"
#include "threads/JSON_read.h"
#include "threads/JSON_write.h"
#include "threads/PID.h"
// end thread includes

// begin transport includes
// end transport includes

// begin filter includes
// end filter includes

// begin other includes
#include "boat_containers.h"
#include <boost/asio/io_service.hpp>
#include <boost/system/error_code.hpp>
#include <boost/asio/serial_port.hpp>
#include <memory>
#include <chrono>
#include <thread>

// END DO NOT DELETE THIS SECTION

const std::string default_broadcast ("192.168.1.255:15000");
// default transport settings
std::string host ("");
const std::string default_multicast ("239.255.0.1:4150");
madara::transport::QoSTransportSettings settings;

// create shortcuts to MADARA classes and namespaces
namespace controllers = gams::controllers;
typedef madara::knowledge::KnowledgeRecord   Record;
typedef Record::Integer Integer;

const std::string KNOWLEDGE_BASE_PLATFORM_KEY (".platform");
bool plat_set = false;
std::string platform ("boat");
std::string algorithm ("null");
std::vector <std::string> accents;

// controller variables
double period (1.0);
//double loop_time (600.0);
double loop_time(-1.0);

#define GAMS_RUN_HZ 1
#define GAMS_SEND_HZ 1

// madara commands from a file
std::string madara_commands = "";

// for setting debug levels through command line
int madara_debug_level (-1);
int gams_debug_level (-1);

// number of agents in the swarm
Integer num_agents (-1);

// file path to save received files to
std::string file_path;

// controller settings are preferred with GAMS now
gams::controllers::ControllerSettings controller_settings;

void print_usage (char * prog_name)
{
  madara_logger_ptr_log (gams::loggers::global_logger.get (),
    gams::loggers::LOG_ALWAYS,
"\nProgram summary for %s:\n\n" 
"     Loop controller setup for gams\n" 
" [-A |--algorithm type]        algorithm to start with\n" 
" [-a |--accent type]           accent algorithm to start with\n" 
" [-b |--broadcast ip:port]     the broadcast ip to send and listen to\n"
" [-cp|--checkpoint-prefix pref the filename prefix for checkpoints\n"
" [-cs|--checkpoint-strategy #  the strategy to use for checkpointing\n"
"                               0: None (default)\n"
"                               1: Every controller loop\n"
"                               2: Every controller send update\n"
" [-d |--domain domain]         the knowledge domain to send and listen to\n" 
" [-e |--rebroadcasts num]      number of hops for rebroadcasting messages\n" 
" [-f |--logfile file]          log to a file\n" 
" [-i |--id id]                 the id of this agent (should be non-negative)\n" 
" [--madara-level level]        the MADARA logger level (0+, higher is higher detail)\n" 
" [--gams-level level]          the GAMS logger level (0+, higher is higher detail)\n" 
" [-L |--loop-time time]        time to execute loop\n"
" [-m |--multicast ip:port]     the multicast ip to send and listen to\n" 
" [-M |--madara-file <file>]    file containing madara commands to execute\n" 
"                               multiple space-delimited files can be used\n" 
" [-n |--num_agents <number>]   the number of agents in the swarm\n" 
" [-o |--host hostname]         the hostname of this process (def:localhost)\n" 
" [-p |--platform type]         platform for loop (vrep, dronerk)\n" 
" [-P |--period period]         time, in seconds, between control loop executions\n" 
" [-q |--queue-length length]   length of transport queue in bytes\n" 
" [-r |--reduced]               use the reduced message header\n" 
" [-t |--target path]           file system location to save received files (NYI)\n" 
" [-u |--udp ip:port]           a udp ip to send to (first is self to bind to)\n" 
"\n",
        prog_name);
  exit (0);
}

// handle command line arguments
void handle_arguments (int argc, char ** argv)
{
  for (int i = 1; i < argc; ++i)
  {
    std::string arg1 (argv[i]);

    if (arg1 == "-A" || arg1 == "--algorithm")
    {
      if (i + 1 < argc && argv[i + 1][0] != '-')
        algorithm = argv[i + 1];
      else
        print_usage (argv[0]);

      ++i;
    }
    else if (arg1 == "-a" || arg1 == "--accent")
    {
      if (i + 1 < argc && argv[i + 1][0] != '-')
        accents.push_back (argv[i + 1]);
      else
        print_usage (argv[0]);

      ++i;
    }
    else if (arg1 == "-b" || arg1 == "--broadcast")
    {
      if (i + 1 < argc && argv[i + 1][0] != '-')
      {
        settings.hosts.push_back (argv[i + 1]);
        settings.type = madara::transport::BROADCAST;
      }
      else
        print_usage (argv[0]);

      ++i;
    }
    else if (arg1 == "-cp" || arg1 == "--checkpoint-prefix")
    {
      if (i + 1 < argc)
      {
        std::stringstream buffer (argv[i + 1]);
        buffer >> controller_settings.checkpoint_prefix;
      }
      else
        print_usage (argv[0]);

      ++i;
    }
    else if (arg1 == "-cs" || arg1 == "--checkpoint-strategy")
    {
      if (i + 1 < argc)
      {
        std::stringstream buffer (argv[i + 1]);
        buffer >> controller_settings.checkpoint_strategy;
      }
      else
        print_usage (argv[0]);

      ++i;
    }
    else if (arg1 == "-d" || arg1 == "--domain")
    {
      if (i + 1 < argc && argv[i + 1][0] != '-')
        settings.write_domain = argv[i + 1];
      else
        print_usage (argv[0]);

      ++i;
    }
    else if (arg1 == "-e" || arg1 == "--rebroadcasts")
    {
      if (i + 1 < argc && argv[i + 1][0] != '-')
      {
        int hops;
        std::stringstream buffer (argv[i + 1]);
        buffer >> hops;

        settings.set_rebroadcast_ttl (hops);
        settings.enable_participant_ttl (hops);
      }
      else
        print_usage (argv[0]);

      ++i;
    }
    else if (arg1 == "-f" || arg1 == "--logfile")
    {
      if (i + 1 < argc && argv[i + 1][0] != '-')
      {
        madara::logger::global_logger->add_file (argv[i + 1]);
        gams::loggers::global_logger->add_file (argv[i + 1]);
      }
      else
        print_usage (argv[0]);

      ++i;
    }

    else if (arg1 == "-i" || arg1 == "--id")
    {
      if (i + 1 < argc && argv[i +1][0] != '-')
      {
        std::stringstream buffer (argv[i + 1]);
        buffer >> settings.id;
      }
      else
        print_usage (argv[0]);

      ++i;
    }
    else if (arg1 == "--madara-level")
    {
      if (i + 1 < argc && argv[i + 1][0] != '-')
      {
        std::stringstream buffer (argv[i + 1]);
        buffer >> madara_debug_level;
      }
      else
        print_usage (argv[0]);

      ++i;
    }
    else if (arg1 == "--gams-level")
    {
      if (i + 1 < argc && argv[i + 1][0] != '-')
      {
        std::stringstream buffer (argv[i + 1]);
        buffer >> gams_debug_level;
      }
      else
        print_usage (argv[0]);

      ++i;
    }
    else if (arg1 == "-L" || arg1 == "--loop-time")
    {
      if (i + 1 < argc && argv[i + 1][0] != '-')
      {
        std::stringstream buffer (argv[i + 1]);
        buffer >> loop_time;

        controller_settings.run_time = loop_time;
      }
      else
        print_usage (argv[0]);

      ++i;
    }
    else if (arg1 == "-m" || arg1 == "--multicast")
    {
      if (i + 1 < argc && argv[i + 1][0] != '-')
      {
        settings.hosts.push_back (argv[i + 1]);
        settings.type = madara::transport::MULTICAST;
      }
      else
        print_usage (argv[0]);

      ++i;
    }
    else if (arg1 == "-M" || arg1 == "--madara-file")
    {
      bool files = false;
      ++i;
      for (;i < argc && argv[i][0] != '-'; ++i)
      {
        std::string filename = argv[i];
        if (madara::utility::file_exists (filename))
        {
          madara_commands += madara::utility::file_to_string (filename);
          madara_commands += ";\n";
          files = true;
        }
      }
      --i;

      if (!files)
        print_usage (argv[0]);
    }
    else if (arg1 == "-n" || arg1 == "--num_agents")
    {
      if (i + 1 < argc && argv[i + 1][0] != '-')
      {
        std::stringstream buffer (argv[i + 1]);
        buffer >> num_agents;
      }
      else
        print_usage (argv[0]);

      ++i;
    }
    else if (arg1 == "-o" || arg1 == "--host")
    {
      if (i + 1 < argc && argv[i + 1][0] != '-')
        host = argv[i + 1];
      else
        print_usage (argv[0]);

      ++i;
    }
    else if (arg1 == "-p" || arg1 == "--platform")
    {
      if (i + 1 < argc && argv[i + 1][0] != '-')
      {
        platform = argv[i + 1];
        plat_set = true;
      }
      else
        print_usage (argv[0]);

      ++i;
    }
    else if (arg1 == "-P" || arg1 == "--period")
    {
      if (i + 1 < argc && argv[i + 1][0] != '-')
      {
        std::stringstream buffer (argv[i + 1]);
        buffer >> period;

        controller_settings.loop_hertz = 1.0 / period;
      }
      else
        print_usage (argv[0]);

      ++i;
    }
    else if (arg1 == "-q" || arg1 == "--queue-length")
    {
      if (i + 1 < argc && argv[i + 1][0] != '-')
      {
        std::stringstream buffer (argv[i + 1]);
        buffer >> settings.queue_length;
      }
      else
        print_usage (argv[0]);

      ++i;
    }
    else if (arg1 == "-r" || arg1 == "--reduced")
    {
      settings.send_reduced_message_header = true;
    }
    else if (arg1 == "-t" || arg1 == "--target")
    {
      if (i + 1 < argc && argv[i + 1][0] != '-')
        file_path = argv[i + 1];
      else
        print_usage (argv[0]);

      ++i;
    }
    else if (arg1 == "-u" || arg1 == "--udp")
    {
      if (i + 1 < argc && argv[i + 1][0] != '-')
      {
        settings.hosts.push_back (argv[i + 1]);
        settings.type = madara::transport::UDP;
      }
      else
        print_usage (argv[0]);

      ++i;
    }
    else
    {
      print_usage (argv[0]);
    }
  }
}

// perform main logic of program
int main (int argc, char ** argv)
{
  settings.type = madara::transport::MULTICAST;
  settings.queue_length = 10000000;

  controller_settings.loop_hertz = GAMS_RUN_HZ;
  controller_settings.send_hertz = GAMS_SEND_HZ;
  controller_settings.run_time = -1;


  // handle all user arguments
  handle_arguments (argc, argv);

  if (settings.hosts.size () == 0)
  {
    // setup default transport as multicast
    settings.hosts.resize (1);
    settings.hosts[0] = default_multicast;
  }
  
  // set this once to allow for debugging knowledge base creation
  if (madara_debug_level >= 0)
  {
    madara::logger::global_logger->set_level (madara_debug_level);
  }

  // create knowledge base and a control loop
  madara::knowledge::KnowledgeBase knowledge;

  // begin on receive filters
  // end on receive filters
  
  // begin on send filters
  // end on send filters
  
  // if you only want to use custom transports, delete following
  knowledge.attach_transport (host, settings);
  
  // begin transport creation 
  // end transport creation
  
  // set this once to allow for debugging controller creation
  if (gams_debug_level >= 0)
  {
    gams::loggers::global_logger->set_level (gams_debug_level);
  }

  controllers::BaseController controller (knowledge, controller_settings);
  madara::threads::Threader threader (knowledge);

  // initialize variables and function stubs
  controller.init_vars (settings.id, num_agents);
  
  std::vector <std::string> aliases;

  // create containers
  Containers containers(knowledge, settings.id);

  boost::asio::io_service io;
  std::shared_ptr<boost::asio::serial_port> port = std::make_shared<boost::asio::serial_port>(io);  
  std::string port_name = EBOARD_PORT_NAME;
  boost::system::error_code ec; 
  bool port_ready = false;
  while (!port_ready)
  {
    madara_logger_ptr_log(gams::loggers::global_logger.get(), 
      gams::loggers::LOG_MAJOR,
      "controller::main:"
      " INFO: Opening serial port to EBoard\n");
    port->open(port_name, ec);
    if (!ec) 
    {
      if (port->is_open()) 
      {
          port->set_option(boost::asio::serial_port_base::baud_rate(EBOARD_BAUD_RATE));
          port_ready = true;
          madara_logger_ptr_log(gams::loggers::global_logger.get(), 
            gams::loggers::LOG_MAJOR,
            "controller::main:"
            " INFO: port->open() succeeded, connected to EBoard\n");
          break;
      }
      else 
      {
      madara_logger_ptr_log(gams::loggers::global_logger.get(), 
        gams::loggers::LOG_MAJOR,
        "controller::main:"
        " ERROR: port->open() returned false\n");
      }
    }
    else 
    {
      madara_logger_ptr_log(gams::loggers::global_logger.get(), 
        gams::loggers::LOG_MAJOR,
        "controller::main:"
        " ERROR: port->open() failed: EBoard not plugged in?\n");
    }
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }
  
  
  // begin adding custom algorithm factories
  // end adding custom algorithm factories

  // begin adding custom platform factories

  // add boat factory
  aliases.clear ();
  aliases.push_back ("boat");

  controller.add_platform_factory (aliases,
    new platforms::boatFactory ());
  // end adding custom platform factories
  
  // read madara initialization
  if (madara_commands != "")
  {
    knowledge.evaluate (madara_commands,
      madara::knowledge::EvalSettings(false, true));
  }

  // set debug levels if they have been set through command line
  if (madara_debug_level >= 0)
  {
    std::stringstream temp_buffer;
    temp_buffer << "agent." << settings.id << ".madara_debug_level = ";
    temp_buffer << madara_debug_level;

    madara::logger::global_logger->clear();
    madara::logger::global_logger->set_level (madara_debug_level);
    madara::logger::global_logger->set_timestamp_format();
    madara::logger::global_logger->add_file ("madara_log.txt");
  
    // modify the debug level being used but don't send out to others
    knowledge.evaluate (temp_buffer.str (),
      madara::knowledge::EvalSettings (true, true));
  }

  if (gams_debug_level >= 0)
  {
    std::stringstream temp_buffer;
    temp_buffer << "agent." << settings.id << ".gams_debug_level = ";
    temp_buffer << gams_debug_level;

    gams::loggers::global_logger->clear();
    gams::loggers::global_logger->set_level (gams_debug_level);
    gams::loggers::global_logger->set_timestamp_format();
    gams::loggers::global_logger->add_file ("gams_log.txt");

    // modify the debug level being used but don't send out to others
    knowledge.evaluate (temp_buffer.str (),
      madara::knowledge::EvalSettings (true, true));
  }

  // initialize the platform and algorithm
  // default to platform in knowledge base if platform not set in command line
  if (!plat_set && knowledge.exists (KNOWLEDGE_BASE_PLATFORM_KEY))
    platform = knowledge.get (KNOWLEDGE_BASE_PLATFORM_KEY).to_string ();
  controller.init_platform (platform);
  controller.init_algorithm (algorithm);

  // check if it is a boat platform via a dynamic_cast not returning null, and if it is a valid pointer, set the containers
  gams::platforms::BasePlatform * platform_ptr = controller.get_platform();  
  platforms::boat * boat_platform_ptr;
  boat_platform_ptr = dynamic_cast< platforms::boat * > (platform_ptr);
  if (boat_platform_ptr != nullptr)
  {
    boat_platform_ptr->set_containers(containers);
  }

  // add any accents
  for (unsigned int i = 0; i < accents.size (); ++i)
  {
    controller.init_accent (accents[i]);
  }

  /**
   * WARNING: the following section will be regenerated whenever new threads
   * are added via this tool. So, you can adjust hertz rates and change how
   * the thread is initialized, but the entire section will be regenerated
   * with all threads in the threads directory, whenever you use the new
   * thread option with the gpc.pl script.
   **/

  // begin thread creation
  threads::localization * localizationThread = new threads::localization(containers);

  threader.run (20.0, "localization", localizationThread);
  threader.run (35.0, "JSON_read", new threads::JSON_read (port, containers, localizationThread));
  threader.run (10.0, "JSON_write", new threads::JSON_write (port, containers));
  threader.run (20.0, "PID", new threads::PID(containers));
  // end thread creation
  
  /**
   * END WARNING
   **/
  
  // run a mape loop for algorithm and platform control
  // controller.run_hz (GAMS_RUN_HZ, loop_time, GAMS_SEND_HZ);
  controller.run ();

  // terminate all threads after the controller
  threader.terminate ();
  
  // wait for all threads
  threader.wait ();
  
  // print all knowledge values
  knowledge.print ();

  return 0;
}

