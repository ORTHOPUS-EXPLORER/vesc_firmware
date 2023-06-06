#include "app_orthopus.h"
#include "commands.h"

static void orthopus_pos_cmd(int argc, const char **argv);
static void orthopus_filter_cmd(int argc, const char **argv);
static void orthopus_offset_cmd(int argc, const char **argv);
static void orthopus_config_cmd(int argc, const char **argv);
static void orthopus_limits_cmd(int argc, const char **argv);
static void orthopus_perf_cmd(int argc, const char **argv);
static void orthopus_control_cmd(int argc, const char **argv);

static void orthopus_cmd_init(void)
{
   terminal_register_command_callback(
    "o_offset",
    "[Orthopus] Initialize Pos PID offset with current AMS (or forced) value. joint: based on AMS zero / encoder: based on actual position  ",
    "[joint/encoder]",
    orthopus_offset_cmd
  );

  terminal_register_command_callback(
    "o_config",
    "[Orthopus] Load/Save Orthopus config from/to EEPROM",
    "[print/dprint/load/save/reset/setrate/enabletimecomp/disabletimecomp]",
    orthopus_config_cmd
  );

  terminal_register_command_callback(
    "o_pos",
    "[Orthopus] Get current positions",
    "",
    orthopus_pos_cmd
  );

  terminal_register_command_callback(
    "o_filter",
    "[Orthopus] AMS filter parameters",
    "[anglestep/enable/disable/enableplot/disableplot/encerrorgain]",
    orthopus_filter_cmd
  );

  terminal_register_command_callback(
    "o_limits",
    "[Orthopus] Actuator limits setting",
    "[posmax/posmin/enable/disable/reachangle/reachspeed]",
    orthopus_limits_cmd
  );

  terminal_register_command_callback(
    "o_perf",
    "[Orthopus] Performance stats",
    "[void/enableplot]",
    orthopus_perf_cmd
  );

  terminal_register_command_callback(
    "o_control",
    "[Orthopus] AMS filter parameters",
    "[enable/disable/enableplot/disableplot/kp/zerotorque/torquefilterconst/enabledeadzone/disabledeadzone/a/demo1]",
    orthopus_control_cmd
  );
  //TODO: o_perf : print performance stats (actual rate, mean rate, jitter, etc.)
}

static void orthopus_cmd_deinit(void)
{
  terminal_unregister_callback(orthopus_offset_cmd);
  terminal_unregister_callback(orthopus_config_cmd);
}

static void orthopus_pos_cmd(int argc, const char **argv)
{
  (void)argc;(void)argv;
  double ams_v     = encoder_cfg_as504x.state.last_enc_angle;//enc_as504x_read_angle(&encoder_cfg_as504x);
  double sincos_v  = enc_sincos_read_deg(&encoder_cfg_sincos);
  double orthop_v  = orthopus_read_encoder();
  double pid_v     = mc_interface_get_pid_pos_now();
  double pid_o     = mc_interface_get_configuration()->p_pid_offset;

  commands_printf("orthopus_config.encoder_offset : % 7.3f", (double)orthopus_config.encoder_offset   );
  commands_printf("PID_pos offset                 : % 7.3f", pid_o                                    );
  commands_printf("AMS_pos                        : % 7.3f", ams_v                                    );
  commands_printf("SINCOS_pos                     : % 7.3f", sincos_v                                 );
  commands_printf("orthopus_read_encoder()        : % 7.3f", orthop_v                                 );
  commands_printf("PID_pos_now                    : % 7.3f", pid_v                                    );
  commands_printf("pos_multiturn_now              : % 7.3f", (double)orthopus_state.pos_multiturn_now );
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
  } else {
    commands_printf("Invalid arguments.");
  }
}

static void orthopus_config_cmd(int argc, const char **argv)
{
  if(argc == 1)
  {
    commands_printf("Invalid arguments.");
    return;
  }
  float val = 0; //store
  if (argc == 3)
      sscanf(argv[2], "%f", &val);

  if(argc == 0 || !strcmp(argv[1],"print"))
  {
    commands_printf("Encoder offset:              % 7.3f",(double)orthopus_config.encoder_offset                    );
    commands_printf("Encoder filter anglestep:    % 7.3f",(double)orthopus_config.encoder_filter_anglestep          );
    commands_printf("Encoder Filter enabled:      %s", orthopus_config.encoder_filter_enable ? "true" : "false"     );
    commands_printf("Encoder Filter plot enabled: %s", orthopus_config.encoder_filter_plot_enable ? "true" : "false");
    commands_printf("Limits enabled:              %s", orthopus_config.limits_enable ? "true" : "false"             );
    commands_printf("Config set:                  %s", orthopus_config.orthopus_config_set ? "true" : "false"       );
    commands_printf("Limits pos max:              % 7.3f",(double)orthopus_config.limits_pos_max                    );
    commands_printf("Limits pos min:              % 7.3f",(double)orthopus_config.limits_pos_min                    );
    commands_printf("Limits reach angle:          % 7.3f",(double)orthopus_config.limits_reach_angle                );
    commands_printf("Limits reach speed:          % 7.3f",(double)orthopus_config.limits_reach_speed                );
    commands_printf("Encoder filter error gain:   % 7.3f",(double)orthopus_config.encoder_filter_error_gain         );
    commands_printf("Loop rate:                   % 5d",(int)orthopus_config.rate_hz                                );
  }
  else if(!strcmp(argv[1],"dprint"))
  {
    const uint32_t sz = sizeof(orthopus_config_t)/4;
    commands_printf("Bool: %d uint8_t: %d int: %d Uint32: %d Float: %d ", sizeof(bool), sizeof(uint8_t), sizeof(int), sizeof(uint32_t), sizeof(float));
    commands_printf("Cfg: Print %d dwords",sz); 
    uint32_t addr=0;
    for(addr=0;addr<sz;addr++)
    {
      uint32_t* raddr = (uint32_t*)((uint8_t*)(&orthopus_config)+addr*4);
      eeprom_var v; v.as_u32 = *raddr;
      commands_printf("  [0x%02X][0x%08p] '0x%04X/% 5d/% 5.3f'",addr,raddr,v.as_u32,v.as_i32,(double)v.as_float);
    }
  }
  else if(!strcmp(argv[1],"reset"))
  {
    orthopus_config_reset(&orthopus_config);
    commands_printf("Orthopus config reset to default. Don't forget to save to EEPROM !");
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
    orthopus_config.orthopus_config_set = true;
    if(orthopus_config_save(&orthopus_config))
      commands_printf("Orthopus config saved to EEPROM");
    else
      commands_printf("Orthopus config save failed =/");
  } else if(!strcmp(argv[1],"setrate"))
  {
    if (val > 10) {
      orthopus_config.rate_hz = (int)val;
      commands_printf("rate: % 7.3f", (double)(int)val);
    }

  } else if(!strcmp(argv[1],"enabletimecomp"))
  {
    orthopus_config.perf_compensateexectime = true;
    commands_printf("Execution time compensation Enabled");
  }
  else if(!strcmp(argv[1],"disabletimecomp"))
  {
    orthopus_config.perf_compensateexectime = false;
    commands_printf("Execution time compensation Disabled");
  } else {
    commands_printf("Invalid arguments.");
  }
} //orthopus_config.perf_compensatelag

static void orthopus_filter_cmd(int argc, const char **argv)
{
  if(argc == 1)
  {
    commands_printf("Invalid arguments.");
    return;
  }
  float v = 0; //store
  if (argc == 3)
      sscanf(argv[2], "%f", &v);

  if(!strcmp(argv[1],"anglestep"))
  {
    orthopus_config.encoder_filter_anglestep = v;
    commands_printf("Update filter angle step: % 7.3f", (double)v);
  }
  else if(!strcmp(argv[1],"enable"))
  {
    orthopus_config.encoder_filter_enable = true;
    commands_printf("Encoder filter Enabled");
  }
  else if(!strcmp(argv[1],"disable"))
  {
    orthopus_config.encoder_filter_enable = false;
    commands_printf("Encoder filter Disabled");
  }
  else if(!strcmp(argv[1],"enableplot"))
  {
    orthopus_config.encoder_filter_plot_enable = true;
    commands_printf("Encoder filter plot Enabled");
  }
  else if(!strcmp(argv[1],"disableplot"))
  {
    orthopus_config.encoder_filter_plot_enable = false;
    commands_printf("Encoder filter plot Disabled");
  }
  else if(!strcmp(argv[1],"encerrorgain"))
  {
    orthopus_config.encoder_filter_error_gain = v;
    commands_printf("Encoder error gain: % 7.3f", (double)v);
  } else {
    commands_printf("Invalid arguments.");
  }
}

static void orthopus_limits_cmd(int argc, const char **argv)
{
  if(argc == 1)
  {
    commands_printf("Invalid arguments.");
    return;
  }
  float v = 0; //store
  if (argc == 3)
      sscanf(argv[2], "%f", &v);

  if(!strcmp(argv[1],"posmax"))
  {
    orthopus_config.limits_pos_max = v;
    commands_printf("Update max pos: % 7.3f", (double)v);
  }
  else if(!strcmp(argv[1],"posmin"))
  {
    orthopus_config.limits_pos_min = v;
    commands_printf("Update min pos: % 7.3f", (double)v);
  }
  else if(!strcmp(argv[1],"enable"))
  {
    orthopus_config.limits_enable = true;
    commands_printf("limits Enabled");
  }
  else if(!strcmp(argv[1],"disable"))
  {
    orthopus_config.limits_enable = false;
    commands_printf("limits Disabled");
  }
  else if(!strcmp(argv[1],"reachangle"))
  {
    orthopus_config.limits_reach_angle = v;
    commands_printf("Limits reach angle: % 7.3f", (double)v);
  }
  else if(!strcmp(argv[1],"reachspeed"))
  {
    orthopus_config.limits_reach_speed = v;
    commands_printf("Limits reach speed: % 7.3f", (double)v);
  } else {
    commands_printf("Invalid arguments.");
  }
}

static void orthopus_perf_cmd(int argc, const char **argv)
{
  if(argc == 1)
  {
    (void)argc;(void)argv;
    commands_printf("measured period (us) : % f", (double)orthopus_state.time_diff);
    commands_printf("loop execution time (ticks) : % d", orthopus_state.exectime);
    commands_printf("requested rade (Hz) : % 7.3f", (double)orthopus_config.rate_hz);
    if (orthopus_state.time_diff != 0)
      commands_printf("measured rate (Hz) : % 7.3f", (double)(1.0/(orthopus_state.time_diff/1000000.0)));
    commands_printf("measured mean period  (us) : % 7.3f", (double)orthopus_state.time_diff_filt);
    if (orthopus_state.time_diff_filt != 0)
      commands_printf("measured mean rate (Hz) : % 7.3f", (double)(1.0/(orthopus_state.time_diff_filt/1000000.0)));
    commands_printf("max period since last call of o_perf (us) : % d", orthopus_state.maxperiod);
    commands_printf("min period since last call of o_perf (us) : % d", orthopus_state.minperiod);
    orthopus_state.maxperiod = 0; //reset max period
    orthopus_state.minperiod = 10000; //reset min period
  } else if (argc == 2){
    if(!strcmp(argv[1],"enableplot"))
    {
      orthopus_state.perfplot= true;
      commands_printf("Perf plot Enabled");
    }
    else if(!strcmp(argv[1],"disableplot"))
    {
      orthopus_state.perfplot = false;
      commands_printf("Perf plot Disabled");
    } else {
      commands_printf("Invalid arguments.");
    }
  }
}

static void orthopus_control_cmd(int argc, const char **argv)
{
  if(argc == 1)//TODO: print all parameter values
  {
    commands_printf("Invalid arguments.");
    return;
  }
  float v = 0; //store
  if (argc == 3)
      sscanf(argv[2], "%f", &v);

  if(!strcmp(argv[1],"enable"))
  {
    orthopus_state.ctrl_enable = true;
    commands_printf("Control Enabled");
  }
  else if(!strcmp(argv[1],"disable"))
  {
    orthopus_state.ctrl_enable = false;
    mc_interface_release_motor();   //disable motor
    mc_interface_ignore_input(100);  // disable new inputs for at least 1 cycle (100ms)
    commands_printf("Control Disabled"); //todo set zero torque and/or estop
  }
  else if(!strcmp(argv[1],"enableplot"))
  {
    orthopus_state.ctrl_plot = true;
    commands_printf("Control plot Enabled");
  }
  else if(!strcmp(argv[1],"disableplot"))
  {
    orthopus_state.ctrl_plot = false;
    commands_printf("Control plot Disabled");
  }
  else if(!strcmp(argv[1],"enabledeadzone"))
  {
    orthopus_state.deadzone = true;
    commands_printf("Deadzone Enabled");
  }
  else if(!strcmp(argv[1],"disabledeadzone"))
  {
    orthopus_state.deadzone = false;
    commands_printf("Deadzone Disabled");
  }
  else if(!strcmp(argv[1],"a"))
  {
    if (v != 0)
    {
      orthopus_state.a = v;
      commands_printf("deadzone a factor: % 7.3f", (double)v);
    } else {
      commands_printf("error: deadzone a factor can't be null");
    }
  }
  else if(!strcmp(argv[1],"demo1"))
  {
    //zero torque
    mc_interface_release_motor();   //disable motor
    mc_interface_ignore_input(1000);
    orthopus_state.ADC3init = false;
    orthopus_state.ADC3zero = 0;
    //setup
    orthopus_state.ctrl_kp = 50;
    orthopus_state.a = 1;
    orthopus_state.deadzone = true;
    orthopus_state.torque_filter_const = 0.1;
    orthopus_state.ctrl_enable = true;
    commands_printf("Configured demo 1: kp50 a1 filterconst0.1 deadzone zerotorque enable");
  }
  else if(!strcmp(argv[1],"stiffness"))
  {
    orthopus_state.stiffness = v;
    commands_printf("Control stiffness: % 7.3f", (double)v);
  }
  else if(!strcmp(argv[1],"kp"))
  {
    orthopus_state.ctrl_kp = v;
    commands_printf("Control kp: % 7.3f", (double)v);
  }
  else if(!strcmp(argv[1],"zerotorque"))
  {
    mc_interface_release_motor();   //disable motor
    mc_interface_ignore_input(1000);
    orthopus_state.ADC3init = false;
    orthopus_state.ADC3zero = 0;
    commands_printf("reinitializing torque zero: % 7.3f", (double)v);
  }
  else if(!strcmp(argv[1],"torquefilterconst"))
  {
    orthopus_state.torque_filter_const = v;
    commands_printf("Torque filter const: % 7.3f", (double)v);
  } else {
    commands_printf("Invalid arguments.");
  }
}