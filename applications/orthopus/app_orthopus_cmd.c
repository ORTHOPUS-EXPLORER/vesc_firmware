#include "app_orthopus.h"
#include "commands.h"

static void orthopus_pos_cmd(int argc, const char **argv);
static void orthopus_offset_cmd(int argc, const char **argv);
static void orthopus_config_cmd(int argc, const char **argv);

static void orthopus_cmd_init(void)
{
   terminal_register_command_callback(
    "o_offset",
    "[Orthopus] Initialize Pos PID offset with current AMS (or forced) value",
    "[d]",
    orthopus_offset_cmd
  );

  terminal_register_command_callback(
    "o_config",
    "[Orthopus] Load/Save Orthopus config from/to EEPROM",
    "[load/save]",
    orthopus_config_cmd
  );

  terminal_register_command_callback(
    "o_pos",
    "[Orthopus] Get current positions",
    "",
    orthopus_pos_cmd
  );
}

static void orthopus_cmd_deinit(void)
{
  terminal_unregister_callback(orthopus_offset_cmd);
  terminal_unregister_callback(orthopus_config_cmd);
}

static void orthopus_pos_cmd(int argc, const char **argv)
{
  (void)argc;(void)argv;
  double ams_v     = enc_as504x_read_angle(&encoder_cfg_as504x);
  double sincos_v  = enc_sincos_read_deg(&encoder_cfg_sincos);
  double orthop_v  = orthopus_read_encoder();
  double pid_v     = mc_interface_get_pid_pos_now();
  double pid_o     = mc_interface_get_configuration()->p_pid_offset;

  commands_printf("orthopus_config.encoder_offset : % 7.3f", (double)orthopus_config.encoder_offset);
  commands_printf("PID_pos offset                 : % 7.3f", pid_o                                 );
  commands_printf("AMS_pos                        : % 7.3f", ams_v                                 );
  commands_printf("SINCOS_pos                     : % 7.3f", sincos_v                              );
  commands_printf("orthopus_read_encoder()        : % 7.3f", orthop_v                              );
  commands_printf("PID_pos_now                    : % 7.3f", pid_v                                 );
}

static void orthopus_offset_cmd(int argc, const char **argv)
{
  if(argc == 1)
  {
    commands_printf("Invalid arguments. Usage: o_offset <joint|encoder> [v]");
    return;
  }
  float v = 0;
  if (argc == 3)
      sscanf(argv[2], "%f", &v);

  if(!strcmp(argv[1],"joint"))
  {
    v = orthopus_set_joint_offset(v, argc == 3);
    commands_printf("Init Joint (ie: PosPID) offset: % 7.3f", (double)v);
  }
  else if(!strcmp(argv[1],"encoder"))
  {
    v = orthopus_set_encoder_offset(v,argc == 3);
    commands_printf("Init encoder offset: % 7.3f", (double)orthopus_config.encoder_offset);
  }
}

static void orthopus_config_cmd(int argc, const char **argv)
{
  if(argc == 0 || !strcmp(argv[1],"print"))
  {
    commands_printf("Encoder offset: % 7.3f",(double)orthopus_config.encoder_offset);
  }
  else if(!strcmp(argv[1],"load"))
  {
    if(orthopus_config_load(&orthopus_config))
      commands_printf("Orthopus config loaded from EEPROM");
    else
      commands_printf("Orthopus config load failed =/");
  }
  else if(!strcmp(argv[1],"save"))
  {
    if(orthopus_config_save(&orthopus_config))
      commands_printf("Orthopus config saved to EEPROM");
    else
      commands_printf("Orthopus config save failed =/");
  }
}
