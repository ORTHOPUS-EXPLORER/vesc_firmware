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
    "[print/dprint/load/save/reset/setrate/etimecomp/dtimecomp/settorquegain/encoder_max_diff/esampleadc3/dsampleadc3]",
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
    "[anglestep/enable/disable/eplot/dplot/encerrorgain]",
    orthopus_filter_cmd
  );

  terminal_register_command_callback(
    "o_limits",
    "[Orthopus] Actuator limits setting",
    "[posmax/posmin/enable/disable/reachangle/reachspeed/kp/kd/powp/powd]",
    orthopus_limits_cmd
  );

  terminal_register_command_callback(
    "o_perf",
    "[Orthopus] Performance stats",
    "[void/eplot]",
    orthopus_perf_cmd
  );

  terminal_register_command_callback(
    "o_control",
    "[Orthopus] AMS filter parameters",
    "[enable/disable/eplot/dplot/kp/zerotorque/loadedzerotorque/torquefilterconst/edeadzone/ddeadzone/a/demo1/eoverwrite/doverwrite/torquecontrol/]",
    orthopus_control_cmd
  );
  //TODO: help
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

  commands_printf("or_conf.encoder_offset : % 7.3f", (double)or_conf.encoder_offset   );
  commands_printf("PID_pos offset                 : % 7.3f", pid_o                                    );
  commands_printf("AMS_pos                        : % 7.3f", ams_v                                    );
  commands_printf("SINCOS_pos                     : % 7.3f", sincos_v                                 );
  commands_printf("orthopus_read_encoder()        : % 7.3f", orthop_v                                 );
  commands_printf("PID_pos_now                    : % 7.3f", pid_v                                    );
  commands_printf("pos_multiturn_now              : % 7.3f", (double)or_state.pos_multiturn_now );
}

/* -------------------------------------------------------------------------- */
/*                                   OFFSET                                   */
/* -------------------------------------------------------------------------- */
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
    commands_printf("Init encoder offset: % 7.3f", (double)or_conf.encoder_offset);
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
    commands_printf("Encoder offset:              % 7.3f",(double)or_conf.encoder_offset                    );
    commands_printf("Encoder filter anglestep:    % 7.3f",(double)or_conf.encoder_filter_anglestep          );
    commands_printf("Encoder Filter enabled:      %s", or_conf.encoder_filter_enable ? "true" : "false"     );
    commands_printf("Encoder Filter plot enabled: %s", or_conf.encoder_filter_plot_enable ? "true" : "false");
    commands_printf("Limits enabled:              %s", or_conf.limits_enable ? "true" : "false"             );
    commands_printf("Config set:                  %s", or_conf.or_conf_set ? "true" : "false"       );
    commands_printf("Limits pos max:              % 7.3f",(double)or_conf.limits_pos_max                    );
    commands_printf("Limits pos min:              % 7.3f",(double)or_conf.limits_pos_min                    );
    commands_printf("Limits reach angle:          % 7.3f",(double)or_conf.limits_reach_angle                );
    commands_printf("Limits reach speed:          % 7.3f",(double)or_conf.limits_reach_speed                );
    commands_printf("Encoder filter error gain:   % 7.3f",(double)or_conf.encoder_filter_error_gain         );
    commands_printf("Loop rate:                   % 5d",(int)or_conf.perf_rate_hz                                );
    commands_printf("ctrl_torquezero:                  % 7.3f",(double)or_conf.ctrl_torquezero                        );
    commands_printf("ctrl_torquegain:                  % 7.3f",(double)or_conf.ctrl_torquegain                        );
    commands_printf("limits_kp:                   % 7.3f",(double)or_conf.limits_kp                         );
    commands_printf("limits_kd:                   % 7.3f",(double)or_conf.limits_kd                         );
    commands_printf("Limits_powp:                 % 5d",(int)or_conf.limits_powp                            );
    commands_printf("Limits_powd:                 % 5d",(int)or_conf.limits_powd                            );
    commands_printf("limits_damp_reachangle:      % 7.3f",(double)or_conf.limits_damp_reachangle            );
    commands_printf("Stiffness:                   % 7.3f", (double)or_conf.ctrl_stiffness                   );
    commands_printf("damping:                     % 7.3f", (double)or_conf.ctrl_damping                     );
    commands_printf("encoder_max_diff:                % 7.3f", (double)or_conf.encoder_max_diff                     );
    commands_printf("adc3 sampled in mc pwm callback: %s", or_conf.ctrl_sample_adc3 ? "true" : "false"       );
  }
  else if(!strcmp(argv[1],"dprint"))
  {
    const uint32_t sz = sizeof(orthopus_config_t)/4;
    commands_printf("Bool: %d uint8_t: %d int: %d Uint32: %d Float: %d ", sizeof(bool), sizeof(uint8_t), sizeof(int), sizeof(uint32_t), sizeof(float));
    commands_printf("Cfg: Print %d dwords",sz); 
    uint32_t addr=0;
    for(addr=0;addr<sz;addr++)
    {
      uint32_t* raddr = (uint32_t*)((uint8_t*)(&or_conf)+addr*4);
      eeprom_var v; v.as_u32 = *raddr;
      commands_printf("  [0x%02X][0x%08p] '0x%04X/% 5d/% 5.3f'",addr,raddr,v.as_u32,v.as_i32,(double)v.as_float);
    }
  }
  else if(!strcmp(argv[1],"reset"))
  {
    orthopus_config_reset(&or_conf);
    commands_printf("Orthopus config reset to default. Don't forget to save to EEPROM !");
  }
  else if(!strcmp(argv[1],"load"))
  {
    if(orthopus_config_load(&or_conf))
      commands_printf("Orthopus config loaded from EEPROM");
    else
      commands_printf("Orthopus config load failed =/");
  }
  else if(!strcmp(argv[1],"save"))
  {
    or_conf.or_conf_set = true;
    if(orthopus_config_save(&or_conf))
      commands_printf("Orthopus config saved to EEPROM");
    else
      commands_printf("Orthopus config save failed =/");
  } else if(!strcmp(argv[1],"setrate"))
  {
    if (val > 10) {
      or_conf.perf_rate_hz = (int)val;
      commands_printf("rate: % 7.3f", (double)(int)val);
    }

  } 
  else if(!strcmp(argv[1],"stiffness"))
  {
    or_conf.ctrl_stiffness = val;
    commands_printf("Control stiffness: % 7.3f", (double)val);
  }
  else if(!strcmp(argv[1],"damping"))
  {
    or_conf.ctrl_damping = val;
    commands_printf("Control damping: % 7.3f", (double)val);
  }
  else if(!strcmp(argv[1],"encoder_max_diff"))
  {
    or_conf.encoder_max_diff = val;
    commands_printf("Max allowed angle between encoders: % 7.3f", (double)val);
  }
  
  else if(!strcmp(argv[1],"etimecomp"))
  {
    or_conf.perf_compensateexectime = true;
    commands_printf("Execution time compensation Enabled");
  }
  else if(!strcmp(argv[1],"dtimecomp"))
  {
    or_conf.perf_compensateexectime = false;
    commands_printf("Execution time compensation Disabled");
  }
  else if(!strcmp(argv[1],"esampleadc3"))
  {
    or_conf.ctrl_sample_adc3 = true;
    commands_printf("Sampling ADC3 in high freq loop");
  }
  else if(!strcmp(argv[1],"dsampleadc3"))
  {
    or_conf.ctrl_sample_adc3 = false;
    commands_printf("Sampling ADC3 in main control loop");
  }
  /*else if(!strcmp(argv[1],"storetorquezero"))
  {
    or_conf.ctrl_torquezero  = or_state.adc3_zero;
    commands_printf("Saved actual torque zero [% 7.3f] to config, don't forget to save config to eeprom", (double)or_conf.ctrl_torquezero);
  }*/
  else if(!strcmp(argv[1],"settorquegain"))
  {
    or_conf.ctrl_torquegain = val;
    commands_printf("ctrl_torquegain: % 7.3f", (double)val);
  } else {
    commands_printf("Invalid arguments.");
  }
}

/* -------------------------------------------------------------------------- */
/*                                   FILTER                                   */
/* -------------------------------------------------------------------------- */
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
    or_conf.encoder_filter_anglestep = v;
    commands_printf("Update filter angle step: % 7.3f", (double)v);
  }
  else if(!strcmp(argv[1],"enable"))
  {
    or_conf.encoder_filter_enable = true;
    commands_printf("Encoder filter Enabled");
  }
  else if(!strcmp(argv[1],"disable"))
  {
    or_conf.encoder_filter_enable = false;
    commands_printf("Encoder filter Disabled");
  }
  else if(!strcmp(argv[1],"eplot"))
  {
    or_conf.encoder_filter_plot_enable = true;
    commands_printf("Encoder filter plot Enabled");
  }
  else if(!strcmp(argv[1],"dplot"))
  {
    or_conf.encoder_filter_plot_enable = false;
    commands_printf("Encoder filter plot Disabled");
  }
  else if(!strcmp(argv[1],"encerrorgain"))
  {
    or_conf.encoder_filter_error_gain = v;
    commands_printf("Encoder error gain: % 7.3f", (double)v);
  } else {
    commands_printf("Invalid arguments.");
  }
}

/* -------------------------------------------------------------------------- */
/*                                   LIMITS                                   */
/* -------------------------------------------------------------------------- */
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
    or_conf.limits_pos_max = v;
    commands_printf("Update max pos: % 7.3f", (double)v);
  }
  else if(!strcmp(argv[1],"posmin"))
  {
    or_conf.limits_pos_min = v;
    commands_printf("Update min pos: % 7.3f", (double)v);
  }
  else if(!strcmp(argv[1],"enable"))
  {
    or_conf.limits_enable = true;
    commands_printf("limits Enabled");
  }
  else if(!strcmp(argv[1],"disable"))
  {
    or_conf.limits_enable = false;
    commands_printf("limits Disabled");
  }
  else if(!strcmp(argv[1],"reachangle"))
  {
    or_conf.limits_reach_angle = v;
    commands_printf("Limits reach angle: % 7.3f", (double)v);
  }
  else if(!strcmp(argv[1],"reachspeed"))
  {
    or_conf.limits_reach_speed = v;
    commands_printf("Limits reach speed: % 7.3f", (double)v);
  }
  else if(!strcmp(argv[1],"kp"))
  {
    or_conf.limits_kp = v;
    commands_printf("Limits kp: % 7.3f", (double)v);
  }
  else if(!strcmp(argv[1],"kd"))
  {
    or_conf.limits_kd = v;
    commands_printf("Limits kd: % 7.3f", (double)v);
  }
  else if(!strcmp(argv[1],"powp"))
  {
    or_conf.limits_powp = (int)v;
    commands_printf("Limits powp: % 5d", (int)v);
  }
  else if(!strcmp(argv[1],"powd"))
  {
    or_conf.limits_powd = (int)v;
    commands_printf("Limits powd: % 5d", (int)v);
  }
  else if(!strcmp(argv[1],"damp_reachangle"))
  {
    or_conf.limits_damp_reachangle = (float)v;
    commands_printf("damp reachangle: % 7.3f", (double)v);
  } else {
    commands_printf("Invalid arguments.");
  }
}

/* -------------------------------------------------------------------------- */
/*                                    PERF                                    */
/* -------------------------------------------------------------------------- */
static void orthopus_perf_cmd(int argc, const char **argv)
{
  if(argc == 1)
  {
    (void)argc;(void)argv;
    commands_printf("measured period (us) : % f", (double)or_state.time_diff);
    commands_printf("loop execution time (ticks) : % d", or_state.perf_exec_time);
    commands_printf("requested rade (Hz) : % 7.3f", (double)or_conf.perf_rate_hz);
    if (or_state.time_diff != 0)
      commands_printf("measured rate (Hz) : % 7.3f", (double)(1.0/(or_state.time_diff/1000000.0)));
    commands_printf("measured mean period  (us) : % 7.3f", (double)or_state.time_diff_filt);
    if (or_state.time_diff_filt != 0)
      commands_printf("measured mean rate (Hz) : % 7.3f", (double)(1.0/(or_state.time_diff_filt/1000000.0)));
    commands_printf("max period since last call of o_perf (us) : % d", or_state.perf_max_period);
    commands_printf("min period since last call of o_perf (us) : % d", or_state.perf_min_period);
    or_state.perf_max_period = 0; //reset max period
    or_state.perf_min_period = 10000; //reset min period
  } else if (argc == 2){
    if(!strcmp(argv[1],"eplot"))
    {
      or_state.perf_plot= true;
      commands_printf("Perf plot Enabled");
    }
    else if(!strcmp(argv[1],"dplot"))
    {
      or_state.perf_plot = false;
      commands_printf("Perf plot Disabled");
    } else {
      commands_printf("Invalid arguments.");
    }
  }
}

/* -------------------------------------------------------------------------- */
/*                                   CONTROL                                  */
/* -------------------------------------------------------------------------- */
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
    or_state.ctrl_enable = true;
    commands_printf("Control Enabled");
  }
  else if(!strcmp(argv[1],"disable"))
  {
    or_state.ctrl_enable = false;
    mc_interface_release_motor();   //disable motor
    mc_interface_ignore_input(100);  // disable new inputs for at least 1 cycle (100ms)
    commands_printf("Control Disabled"); //todo set zero torque and/or estop
  }
  else if(!strcmp(argv[1],"eoverwrite"))
  {
    or_state.ctrl_overwrite = true;

  }
  else if(!strcmp(argv[1],"doverwrite"))
  {
    or_state.ctrl_overwrite = false;
  }
  else if(!strcmp(argv[1],"eplot"))
  {
    or_state.ctrl_plot = true;
    commands_printf("Control plot Enabled");
  }
  else if(!strcmp(argv[1],"dplot"))
  {
    or_state.ctrl_plot = false;
    commands_printf("Control plot Disabled");
  }
  else if(!strcmp(argv[1],"edeadzone"))
  {
    or_conf.ctrl_deadzone = true;
    commands_printf("Deadzone Enabled");
  }
  else if(!strcmp(argv[1],"ddeadzone"))
  {
    or_conf.ctrl_deadzone = false;
    commands_printf("Deadzone Disabled");
  }
  else if(!strcmp(argv[1],"a"))
  {
    if (v != 0)
    {
      or_conf.ctrl_a= v;
      commands_printf("ctrl_deadzone a factor: % 7.3f", (double)v);
    } else {
      commands_printf("error: ctrl_deadzone a factor can't be null");
    }
  }
  else if(!strcmp(argv[1],"demo1"))
  {
    mc_interface_release_motor();   //disable motor
    mc_interface_ignore_input(1000);
    or_conf.ctrl_kp = 4;
    or_conf.ctrl_a= 1;
    or_conf.ctrl_deadzone = true;
    or_conf.torque_filter_const = 0.1;
    or_state.ctrl_enable = true;
    or_conf.ctrl_stiffness = 0.0;
    commands_printf("Configured demo 1: kp4 a1 filterconst0.1 ctrl_deadzone zerotorque enable stiffness 0.0");
  }
  else if(!strcmp(argv[1],"demo2"))
  {
    mc_interface_release_motor();   //disable motor
    mc_interface_ignore_input(1000);
    or_conf.ctrl_kp = 4;
    or_conf.ctrl_a= 3;
    or_conf.ctrl_deadzone = true;
    or_conf.torque_filter_const = 0.1;
    or_state.ctrl_enable = true;
    or_conf.ctrl_stiffness = 0.0;
    commands_printf("Configured demo 2: kp4 a 3 filterconst0.1 ctrl_deadzone zerotorque enable stiffness 0.0");
  }
  else if(!strcmp(argv[1],"torquecontrol"))
  {
    mc_interface_release_motor();   //disable motor
    mc_interface_ignore_input(1000);
    or_state.ctrl_enable = true;
    or_state.ctrl_overwrite = true;
    commands_printf("Overwriting current setpoints into torque setpoint");
  }
  else if(!strcmp(argv[1],"kp"))
  {
    or_conf.ctrl_kp = v;
    commands_printf("Control kp: % 7.3f", (double)v);
  }
  else if(!strcmp(argv[1],"kd"))
  {
    or_conf.ctrl_kd = v;
    commands_printf("Control kd: % 7.3f", (double)v);
  }
  else if(!strcmp(argv[1],"kdfilter"))
  {
    or_conf.ctrl_kd_filter = v;
    commands_printf("Control kd filter const: % 7.3f", (double)v);
  }
  else if(!strcmp(argv[1],"zerotorque"))
  {
    mc_interface_release_motor();   //disable motor
    mc_interface_ignore_input(1000);
    or_state.adc3_init = false;
    or_state.adc3_zero = 0;
    commands_printf("reinitializing torque zero");
  }
  else if(!strcmp(argv[1],"loadedzerotorque"))
  {
    mc_interface_release_motor();   //disable motor
    commands_printf("enabled brake current %7.3f",(double)v);
    mc_interface_set_brake_current(v);
    or_state.adc3_init = false;
    or_state.adc3_zero = 0;
    commands_printf("reinitializing torque zero");
  }
  else if(!strcmp(argv[1],"torquefilterconst"))
  {
    or_conf.torque_filter_const = v;
    commands_printf("torque_now filter const: % 7.3f", (double)v);
  }
  else if(!strcmp(argv[1],"print"))
  {
    commands_printf("Control enabled:     %s", or_state.ctrl_enable   ? "true" : "false" );
    commands_printf("Plot enabled:        %s", or_state.ctrl_plot     ? "true" : "false" );
    commands_printf("Deadzone enabled:    %s", or_conf.ctrl_deadzone     ? "true" : "false" );
    commands_printf("torque_now filter const: % 7.3f", (double)or_conf.torque_filter_const );
    commands_printf("Kp:                  % 7.3f", (double)or_conf.ctrl_kp             );
    commands_printf("Kd:                  % 7.3f", (double)or_conf.ctrl_kd             );
    commands_printf("a:                   % 7.3f", (double)or_conf.ctrl_a                   );
    commands_printf("Stiffness:           % 7.3f", (double)or_conf.ctrl_stiffness      );
    commands_printf("ctrl_torquezero:          % 7.3f", (double)or_state.adc3_zero             );
    commands_printf("Control overwrite:  %s", or_state.ctrl_overwrite ? "true" : "false" );
    commands_printf("control_command:     % 7.3f", (double)or_state.ctrl_command         );
    commands_printf("last_control_command:% 7.3f", (double)or_state.last_ctrl_command    );
    commands_printf("limit_reaction:       % 7.3f", (double)or_state.limit_reaction        );
    commands_printf("ext_pos_setpoint:    % 7.3f", (double)or_state.ext_pos_setpoint     );
    commands_printf("ext_torque_setpoint: % 7.3f", (double)or_state.ext_torque_setpoint  );
    commands_printf("damping:             % 7.3f", (double)or_conf.ctrl_damping        );
    commands_printf("safety stopped:      %s", or_state.stopped       ? "true" : "false" );
    commands_printf("nid1:                % 5d",(int)or_state.nid1                       );
  } else {
    commands_printf("Invalid arguments.");
  }
}