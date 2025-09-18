#include "app_orthopus.h"
#include "commands.h"

void orthopus_pos_cmd(int argc, const char **argv);
//void orthopus_filter_cmd(int argc, const char **argv);
void orthopus_offset_cmd(int argc, const char **argv);
void orthopus_config_cmd(int argc, const char **argv);
//void orthopus_limits_cmd(int argc, const char **argv);
void orthopus_perf_cmd(int argc, const char **argv);
void orthopus_control_cmd(int argc, const char **argv);
void orthopus_comm_cmd(int argc, const char **argv);
void orthopus_can_cmd(int argc, const char **argv);
void OR_SAFETY_cmd(int argc, const char **argv);

void or_cmd_init(void)
{
   terminal_register_command_callback(
    "o_offset",
    "[OR] ... ",
    "[joint/encoder]",
    orthopus_offset_cmd
  );

  terminal_register_command_callback(
    "o_config",
    "[OR] ...",
    "[print/dprint/load/save/reset/setrate/stiffness/damping/etimecomp/dtimecomp/settorquegain/encoder_max_diff/esampleadc3/dsampleadc3]",
    orthopus_config_cmd
  );

  terminal_register_command_callback(
    "o_pos",
    "[OR] ...",
    "",
    orthopus_pos_cmd
  );

  /*terminal_register_command_callback(
    "o_filter",
    "[OR] AMS filter parameters",
    "[anglestep/enable/disable/eplot/dplot/encerrorgain]",
    orthopus_filter_cmd
  );*/

  /*terminal_register_command_callback(
    "o_limits",
    "[OR] Actuator limits setting",
    "[posmax/posmin/enable/disable/reachangle/reachspeed/kp/kd/powp/powd/damp_reachangle]",
    orthopus_limits_cmd
  );*/

  
  terminal_register_command_callback(
    "o_perf",
    "[OR] ...",
    "[-/void/eplot]",
    orthopus_perf_cmd
  );

  terminal_register_command_callback(
    "o_control",
    "[OR] ...",
    "[print/enable/disable/eplot/dplot/kp/zerotorque/loadedzerotorque/torquefilterconst/edeadzone/ddeadzone/a/demo1/demo2/eoverwrite/doverwrite/torquecontrol/setzerotorque/readzerotorque/]",
    orthopus_control_cmd
  );
  //TODO: o_perf : print performance stats (actual rate, mean rate, jitter, etc.)

  terminal_register_command_callback(
    "o_comm",
    "[OR] ...",
    "[stream_rate/set_qd/set_td/process_rx/process_ctrl/print/set_ctrl]",
    orthopus_comm_cmd
  );

  terminal_register_command_callback(
    "o_can",
    "[OR] ...",
    "[tx_eid/tx_sid/tx_b]",
    orthopus_can_cmd
  );

  terminal_register_command_callback(
    "o_safety",
    "[OR] ...",
    "[errors/set_err_warn/set_err_hold/set_err_brake/set_err_estop/clear_test_err/set_enable/set_idle/set_brake/set_hold/set_estop/set_init/clear_all/clear_history/release/print]",
    OR_SAFETY_cmd
  );
}

void or_cmd_deinit(void)
{
  terminal_unregister_callback(orthopus_offset_cmd);
  terminal_unregister_callback(orthopus_config_cmd);
  terminal_unregister_callback(orthopus_pos_cmd);
  //terminal_unregister_callback(orthopus_filter_cmd);
  //terminal_unregister_callback(orthopus_limits_cmd);
  terminal_unregister_callback(orthopus_perf_cmd);
  terminal_unregister_callback(orthopus_comm_cmd);
  terminal_unregister_callback(orthopus_can_cmd);
  terminal_unregister_callback(OR_SAFETY_cmd);
}


//void comm_can_transmit_eid(uint32_t id, const uint8_t *data, uint8_t len);
//void comm_can_transmit_sid(uint32_t id, const uint8_t *data, uint8_t len);
//void comm_can_send_buffer(uint8_t controller_id, uint8_t *data, unsigned int len, uint8_t send);
//bool comm_can_ping(uint8_t controller_id, HW_TYPE *hw_type);
// https://www.vesc-project.com/sites/default/files/imce/u15301/VESC6_CAN_CommandsTelemetry.pdf
void orthopus_can_cmd(int argc, const char **argv)
{
  uint8_t buffer[16] = {0};
  size_t len=0;
  if(argc == 1)
    return;
  // Expecting:
  //  can0  00000179   [2]  01 42
  buffer[0] = 0x01;
  buffer[1] = 0x42;
  len = 2;
  if(argc >= 3)
    sscanf(argv[2],"%d",&len);
  len = len > 16 ? 16 : len;

  // Lower 8 bits are VESC Board IDs
  if(!strcmp(argv[1],"tx_eid")) // 29b IDs, caped to 8 bytes
  {
    comm_can_transmit_eid(0x179, buffer, len);
  }
  else if(!strcmp(argv[1],"tx_sid"))  // 11b IDs, caped to 8 bytes
  {
    comm_can_transmit_sid(0x179, buffer, len);
  }
  else if(!strcmp(argv[1],"tx_b"))  // 29b IDs, 
  {
    // 0: Packet goes to commands_process_packet of receiver
    // 1: Packet goes to commands_send_packet of receiver
    // 2: Packet goes to commands_process and send function is set to null
    //    so that no reply is sent back.
    uint8_t send = 0;
    int r =0;
    if(argc >= 4)
      sscanf(argv[3],"%d",&r);
    send = r > 2 ? 2 : r&0xFF;
    //commands_printf("Len: %d, Send: %d", len, send);
    comm_can_send_buffer(0x79, buffer, len, send);

    // o_can tx_b 1:
    //      can0  00000879   [3]  16 00 01
    // o_can tx_b 6:
    //      can0  00000879   [8]  16 00 01 42 00 00 00 00
    // o_can tx_b 7:
    //      can0  00000579   [8]  00 01 42 00 00 00 00 00
    //      can0  00000779   [6]  16 00 00 07 59 31
    // o_can tx_b 10:
    //      can0  00000579   [8]  00 01 42 00 00 00 00 00
    //      can0  00000579   [4]  07 00 00 00
    //      can0  00000779   [6]  16 00 00 0A F6 FB
    // O_can_tx_b 16:
    //      can0  00000579   [8]  00 01 42 00 00 00 00 00
    //      can0  00000579   [8]  07 00 00 00 00 00 00 00
    //      can0  00000579   [3]  0E 00 00
    //      can0  00000779   [6]  16 00 00 10 CF F4s
    // o_can tx_b  6 1  
    //      can0  00000879   [8]  16 01 01 42 00 00 00 00
  }

}

void orthopus_comm_cmd(int argc, const char **argv)
{
  if(argc == 3 && !strcmp(argv[1],"stream_rate"))
  {
    unsigned int v = 0;
    sscanf(argv[2], "%u", &v);
    if(v < 1000)
      or_conf.stream_rate_10 = v/10;
    commands_printf("Stream Rate: %dHz", ((uint16_t)or_conf.stream_rate_10)*10);
  }
  /*else if(argc == 3 &&!strcmp(argv[1],"set_qd"))
  {
    or_comm_control_t* ctrl = (or_comm_control_t*)or_comm.ctrl;
    float v = 0;
    sscanf(argv[2], "%f", &v);
    if(v >= -360 && v <= 360)
      ctrl->pos = v;
    commands_printf("New qd: %f", (double)ctrl->pos);
  }*/
  else if(argc == 3 &&!strcmp(argv[1],"set_td"))
  {
    or_comm_control_t* ctrl = (or_comm_control_t*)or_comm.ctrl;
    float v = 0;
    sscanf(argv[2], "%f", &v);
    if(v >= -360 && v <= 360)
      ctrl->trq = v;
    commands_printf("New td: %f", (double)ctrl->trq);
  }
  /*else if(argc == 3 && !strcmp(argv[1],"process_rx"))
  {
    or_comm.process_rx = !strcmp(argv[2],"on");
    commands_printf("Process RX: %s", or_comm.process_rx ? "true" : "false");
  }
  else if(argc == 3 && !strcmp(argv[1],"process_ctrl"))
  {
    or_comm.process_ctrl = !strcmp(argv[2],"on");
    commands_printf("Process Ctrl: %s", or_comm.process_ctrl ? "true" : "false");
  }*/
  else if(argc == 2  && !strcmp(argv[1],"print"))
  {
    or_comm_control_t* ctrl = (or_comm_control_t*)or_comm.ctrl;
    commands_printf("Last RX:");
    commands_printf("  Control word: 0x%04X",        ctrl->word);
    commands_printf("  Position    :  % 9.5f",(double)ctrl->pos);
    commands_printf("  Velocity    :  % 9.5f",(double)ctrl->vel);
    commands_printf("  Torque      :  % 9.5f",(double)ctrl->trq);
    commands_printf("Process Ctrl  : %s", or_comm.process_ctrl ? "true" : "false");
    commands_printf("Process RX    : %s", or_comm.process_rx ? "true" : "false");
    or_comm_state_t* st = (or_comm_state_t*)or_comm.state;
    commands_printf("Last TX:");
    commands_printf("  Status word : 0x%04X",         st->word);
    commands_printf("  Position    :  % 9.5f",(double)st->pos );
    commands_printf("  Velocity    :  % 9.5f",(double)st->vel );
    commands_printf("  Torque      :  % 9.5f",(double)st->trq );
    commands_printf("Stream Rate   : %dHz", ((uint16_t)or_conf.stream_rate_10)*10);
    commands_printf("Simu mode     : %s", or_conf.simu_mode ? "true" : "false");
    commands_printf("Auto clear errors: %s", or_conf.auto_clear_errors ? "true" : "false");
    //commands_printf("  Temperature :  % 9.5f",(double)st->temp);
    //commands_printf("  Current     :  % 9.5f",(double)st->curr);
  }
  else if(argc == 3 &&!strcmp(argv[1],"set_ctrl"))
  {
    or_comm_control_t* ctrl = (or_comm_control_t*)or_comm.ctrl;
    uint16_t v;
    sscanf(argv[2], "%hx", &v);
    ctrl->word = v;
    commands_printf("New ctrl_word: 0x%04X", ctrl->word);
  }
  else
    commands_printf("o_comm <print|stream_rate <0-999(in Hz)>|process_ctrl <on|off>|"
                    "process_rx <on|off>|simu_mode <on|off>|set_qd <-1.57..1.57>>");
}

void orthopus_pos_cmd(int argc, const char **argv)
{
  (void)argc;(void)argv;
  double ams_v     = encoder_cfg_as504x.state.last_enc_angle;//enc_as504x_read_angle(&encoder_cfg_as504x);
  double sincos_v  = enc_sincos_read_deg(&encoder_cfg_sincos);
  double orthop_v  = or_read_encoder();
  double pid_v     = mc_interface_get_pid_pos_now();
  double pid_o     = mc_interface_get_configuration()->p_pid_offset;

  commands_printf("or_conf.encoder_offset : % 7.3f", (double)or_conf.encoder_offset   );
  commands_printf("PID_pos offset                 : % 7.3f", pid_o                                    );
  commands_printf("AMS_pos                        : % 7.3f", ams_v                                    );
  commands_printf("SINCOS_pos                     : % 7.3f", sincos_v                                 );
  commands_printf("or_read_encoder()        : % 7.3f", orthop_v                                 );
  commands_printf("PID_pos_now                    : % 7.3f", pid_v                                    );
  commands_printf("pos_multiturn_now              : % 7.3f", (double)or_state.pos_multiturn_now );
}

/* -------------------------------------------------------------------------- */
/*                                   OFFSET                                   */
/* -------------------------------------------------------------------------- */
void orthopus_offset_cmd(int argc, const char **argv)
{
  if(argc == 1)
  {
    commands_printf("X. Usage: o_offset <joint|encoder> [v]");
    return;
  }
  float v = 0;
  if (argc == 3)
      sscanf(argv[2], "%f", &v);

  if(!strcmp(argv[1],"joint"))
  {
    v = or_set_joint_offset(v, argc == 3);
    commands_printf("Init Joint (ie: PosPID) offset: % 7.3f", (double)v);
  }
  else if(!strcmp(argv[1],"encoder"))
  {
    v = or_set_encoder_offset(v,argc == 3);
    commands_printf("Init encoder offset: % 7.3f", (double)or_conf.encoder_offset);
  } else {
    commands_printf("X.");
  }
}

void orthopus_config_cmd(int argc, const char **argv)
{
  if(argc == 1)
  {
    commands_printf("X.");
    return;
  }
  float val = 0; //store
  if (argc == 3)
      sscanf(argv[2], "%f", &val);

  if(argc == 1 || !strcmp(argv[1],"print"))
  {
    commands_printf("Encoder offset:              % 7.3f",(double)or_conf.encoder_offset                    );
    //commands_printf("Encoder filter anglestep:    % 7.3f",(double)or_conf.encoder_filter_anglestep          );
    //commands_printf("Encoder Filter enabled:      %s", or_conf.encoder_filter_enable ? "true" : "false"     );
    //commands_printf("Encoder Filter plot enabled: %s", or_conf.encoder_filter_plot_enable ? "true" : "false");
    commands_printf("Limits enabled:              %s", or_conf.limits_enable ? "true" : "false"             );
    commands_printf("Config set:                  %s", or_conf.signature == ORTHOPUS_CONFIG_T_SIGNATURE ? "true" : "false"       );
    commands_printf("Limits pos max:              % 7.3f",(double)or_conf.limits_pos_max                    );
    commands_printf("Limits pos min:              % 7.3f",(double)or_conf.limits_pos_min                    );
    commands_printf("Limits reach angle:          % 7.3f",(double)or_conf.limits_reach_angle                );
    commands_printf("Limits reach speed:          % 7.3f",(double)or_conf.limits_reach_speed                );
    //commands_printf("Encoder filter error gain:   % 7.3f",(double)or_conf.encoder_filter_error_gain         );
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
    commands_printf("Cfg: Print %d/%d dwords",sz,MAX_CONFIG_U32_SIZE); 
    uint32_t addr=0;
    for(addr=0;addr<sz;addr++)
    {
      uint32_t* raddr = (uint32_t*)((uint8_t*)(&or_conf)+addr*4);
      eeprom_var v; v.as_u32 = *raddr;
      commands_printf("  [0x%02X][0x%08p] '0x%04X/% 5d/% 5.3f'",addr,raddr,v.as_u32,v.as_i32,(double)v.as_float);
    }
  }
  /*else if(!strcmp(argv[1],"reset"))
  {
    or_config_set(&or_conf, NULL);
    commands_printf("OR config reset to default. Don't forget to save to EEPROM !");
  }*/
  else if(!strcmp(argv[1],"load"))
  {
    if(or_config_load(&or_conf))
      commands_printf("OR config loaded from EEPROM");
    else
      commands_printf("OR config load failed =/");
  }
  else if(!strcmp(argv[1],"save"))
  {
    if(or_config_save(&or_conf))
      commands_printf("OR config saved to EEPROM");
    else
      commands_printf("OR config save failed =/");
  } else if(!strcmp(argv[1],"setrate"))
  {
    if (val > 10) {
      or_conf.perf_rate_hz = (int)val;
      commands_printf("rate: % 7.3f", (double)(int)val);
    }

  } 
  /*else if(!strcmp(argv[1],"stiffness"))
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
  }*/
  else if(!strcmp(argv[1],"storetorquezero"))
  {
    or_conf.ctrl_torquezero  = or_state.adc3_zero;
    commands_printf("Saved actual torque zero [% 7.3f] to config, don't forget to save config to eeprom", (double)or_conf.ctrl_torquezero);
  }
  /*else if(!strcmp(argv[1],"settorquegain"))
  {
    or_conf.ctrl_torquegain = val;
    commands_printf("ctrl_torquegain: % 7.3f", (double)val);
  }*/ else {
    commands_printf("X.");
  }
}

/* -------------------------------------------------------------------------- */
/*                                   FILTER                                   */
/* -------------------------------------------------------------------------- */
/*
void orthopus_filter_cmd(int argc, const char **argv)
{
  if(argc == 1)
  {
    commands_printf("X.");
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
    commands_printf("X.");
  }
}

*/

/* -------------------------------------------------------------------------- */
/*                                   LIMITS                                   */
/* -------------------------------------------------------------------------- */
/*
void orthopus_limits_cmd(int argc, const char **argv)
{
  if(argc == 1)
  {
    commands_printf("X.");
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
    commands_printf("X.");
  }
}*/

/* -------------------------------------------------------------------------- */
/*                                    PERF                                    */
/* -------------------------------------------------------------------------- */
void orthopus_perf_cmd(int argc, const char **argv)
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
      commands_printf("X.");
    }
  }
}


/* -------------------------------------------------------------------------- */
/*                                   CONTROL                                  */
/* -------------------------------------------------------------------------- */
void orthopus_control_cmd(int argc, const char **argv)
{
  if(argc == 1)//TODO: print all parameter values
  {
    commands_printf("X.");
    return;
  }
  float v = 0; //store
  if (argc == 3)
      sscanf(argv[2], "%f", &v);

  if(!strcmp(argv[1],"enable"))
  {
    or_set_control_mode(OR_STATE_MODE_TRQ);
    or_set_safety_mode(OR_SAFETY_ENABLE);
    commands_printf("Control Enabled");
  }
  else if(!strcmp(argv[1],"disable"))
  {
    or_set_safety_mode(OR_SAFETY_ENABLE);
    or_set_control_mode(OR_STATE_MODE_OFF);
    mc_interface_release_motor();   //disable motor
    mc_interface_ignore_input(100);  // disable new inputs for at least 1 cycle (100ms)
    commands_printf("Control Disabled"); //todo set zero torque and/or estop
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
  /*else if(!strcmp(argv[1],"edeadzone"))
  {
    or_conf.ctrl_deadzone = true;
    commands_printf("Deadzone Enabled");
  }
  else if(!strcmp(argv[1],"ddeadzone"))
  {
    or_conf.ctrl_deadzone = false;
    commands_printf("Deadzone Disabled");
  }*/
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
    or_conf.ctrl_stiffness = 0.0;
    commands_printf("Configured demo 1: kp4 a1 filterconst0.1 ctrl_deadzone zerotorque enable stiffness 0.0");
    or_set_control_mode(OR_STATE_MODE_TRQ);
    or_set_safety_mode(OR_SAFETY_ENABLE);
  }
  /*else if(!strcmp(argv[1],"demo2"))
  {
    mc_interface_release_motor();   //disable motor
    mc_interface_ignore_input(1000);
    or_conf.ctrl_kp = 4;
    or_conf.ctrl_a= 3;
    or_conf.ctrl_deadzone = true;
    or_conf.torque_filter_const = 0.1;
    or_conf.ctrl_stiffness = 0.0;
    commands_printf("Configured demo 2: kp4 a 3 filterconst0.1 ctrl_deadzone zerotorque enable stiffness 0.0");
    or_set_control_mode(OR_STATE_MODE_TRQ);
    or_set_safety_mode(OR_SAFETY_ENABLE);
  }
  else if(!strcmp(argv[1],"torquecontrol"))
  {
    mc_interface_release_motor();   //disable motor
    mc_interface_ignore_input(1000);
    or_set_control_mode(OR_STATE_MODE_TRQ);
    or_set_safety_mode(OR_SAFETY_ENABLE);
    commands_printf("Overwriting current setpoints into torque setpoint");
  }*/
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
  else if(!strcmp(argv[1],"kt"))
  {
    or_conf.ff_torque_constant = v;
    commands_printf("Torque const Kt for feedforward: % 7.3f", (double)v);
  }
  else if(!strcmp(argv[1],"zerotorque"))
  {
    mc_interface_release_motor();   //disable motor
    mc_interface_ignore_input(1000);
    or_state.adc3_init = false;
    or_state.adc3_zero = 0;
    commands_printf("reinitializing torque zero");
  }
  /*else if(!strcmp(argv[1],"loadedzerotorque"))
  {
    mc_interface_release_motor();   //disable motor
    commands_printf("enabled brake current %7.3f",(double)v);
    mc_interface_set_brake_current(v);
    or_state.adc3_init = false;
    or_state.adc3_zero = 0;
    commands_printf("reinitializing torque zero");
  }*/
  else if(!strcmp(argv[1],"torquefilterconst"))
  {
    or_conf.torque_filter_const = v;
    commands_printf("torque_now filter const: % 7.3f", (double)v);
  }
  else if(!strcmp(argv[1],"setzerotorque"))
  {
    or_conf.ctrl_torquezero = v;
    or_state.adc3_zero = v;
    commands_printf("ctrl_torquezero: % 7.3f", (double)v);
  }
  else if(!strcmp(argv[1],"readzerotorque"))
  {
    commands_printf("adc3_read: % 7.3f", (double)or_state.adc3_val);
  }
  else if(!strcmp(argv[1],"print"))
  {
    commands_printf("Plot enabled:        %s", or_state.ctrl_plot     ? "true" : "false" );
    commands_printf("Deadzone enabled:    %s", or_conf.ctrl_deadzone     ? "true" : "false" );
    commands_printf("torque_now filter const: % 7.3f", (double)or_conf.torque_filter_const );
    commands_printf("Kp:                  % 7.3f", (double)or_conf.ctrl_kp             );
    commands_printf("Kd:                  % 7.3f", (double)or_conf.ctrl_kd             );
    commands_printf("a:                   % 7.3f", (double)or_conf.ctrl_a                   );
    commands_printf("Stiffness:           % 7.3f", (double)or_conf.ctrl_stiffness      );
    commands_printf("FF Torque const kt:     % 7.3f", (double)or_conf.ff_torque_constant);
    commands_printf("ctrl_torquezero:          % 7.3f", (double)or_state.adc3_zero             );
    commands_printf("control_command:     % 7.3f", (double)or_state.ctrl_command         );
    commands_printf("last_control_command:% 7.3f", (double)or_state.last_ctrl_command    );
    commands_printf("limit_reaction:       % 7.3f", (double)or_state.limit_reaction        );
    commands_printf("ext_pos_setpoint:    % 7.3f", (double)or_state.ext_pos_setpoint     );
    commands_printf("ext_torque_setpoint: % 7.3f", (double)or_state.ext_torque_setpoint  );
    commands_printf("damping:             % 7.3f", (double)or_conf.ctrl_damping        );
    commands_printf("safety stopped:      %s", or_state.stopped       ? "true" : "false" );
    commands_printf("nid1:                % 5d",(int)or_state.nid1                       );
    commands_printf("turn sincos:         % 5d",(int)or_state.turn_now                   );
    commands_printf("turn encoder:        % 5d",(int)or_state.enc_turn                   );
  } else {
    commands_printf("X.");
  }
}

/* -------------------------------------------------------------------------- */
/*                                   SAFETY                                   */
/* -------------------------------------------------------------------------- */

/*list of errors:
  ERR_NONE = 0,
  // Control-related
  ERR_POS_STEP,
  ERR_VEL_STEP,
  ERR_TRQ_STEP,
  ERR_SAME_CTRL_OUT,
  //Limits:
  ERR_POS_LIMIT,
  ERR_SPEED_LIMIT,
  //Test errors:
  ERR_TST_WARNING,
  ERR_TST_HOLD,
  ERR_TST_BRAKE,
  ERR_TST_ESTOP,
  // Add others...
  ERR_STP_HOLD,
  ERR_CAN_TIMEOUT,
  ERR_MAX_SPEED,
  ERR_COUNT // Always last
} or_error_t;*/

const char* or_error_messages[ERR_COUNT] = {
    "none",
    "Pos stp",
    "Vel stp",
    "Trq stp",
    "Same ctrlout",
    "Limit endstop",
    "max speed toward endstop",
    "Test warning",
    "Test hold",
    "Test brake",
    "Test estop",
    "Pos error in HOLD",
    "Can timeout",
    "Max speed exceeded",
    // Add corresponding error messages here
};

const char* or_error_level_txt[ERR_LEVEL_COUNT] = {
    "none",
    "warning",
    "hold",
    "brake",
    "estop",
    // Add corresponding error levels here
};

void OR_SAFETY_cmd(int argc, const char **argv)
{
    if (argc == 1) {
        commands_printf("Safety commands - TODO");
    } else if (argc == 2) {
        if (!strcmp(argv[1], "errors")) {
            commands_printf("Active errors:");

            for (int i = 0; i < ERR_COUNT; i++) {
                if (or_active_errors[i]) {
                    commands_printf(" - %s (%s)", or_error_messages[i], or_error_level_txt[or_get_error_severity(i)]);
                }
            }

            commands_printf("Registered errors since startup:");

            for (int i = 0; i < ERR_COUNT; i++) {
                if (or_error_triggered[i]) {
                    commands_printf(" - %s (%s)", or_error_messages[i], or_error_level_txt[or_get_error_severity(i)]);
                }
            }
        } else if (!strcmp(argv[1], "set_err_warn"))
        {
          or_raise_error(ERR_TST_WARNING);
        } else if (!strcmp(argv[1], "set_err_hold"))
        {
          or_raise_error(ERR_TST_HOLD);
        } else if (!strcmp(argv[1], "set_err_brake"))
        {
          or_raise_error(ERR_TST_BRAKE);
        } else if (!strcmp(argv[1], "set_err_estop"))
        {
          or_raise_error(ERR_TST_ESTOP);
        } else if (!strcmp(argv[1], "clear_test_err"))
        {
          or_clear_error(ERR_TST_WARNING);
          or_clear_error(ERR_TST_HOLD);
          or_clear_error(ERR_TST_BRAKE);
          or_clear_error(ERR_TST_ESTOP);
        } else if (!strcmp(argv[1], "set_enable"))
        {
          or_set_safety_mode(OR_SAFETY_ENABLE);
        } else if (!strcmp(argv[1], "set_idle"))
        {
          or_set_safety_mode(OR_SAFETY_IDLE);
        } else if (!strcmp(argv[1], "set_hold"))
        {
          or_set_safety_mode(OR_SAFETY_HOLD);
        } else if (!strcmp(argv[1], "set_estop"))
        {
          or_set_safety_mode(OR_SAFETY_ESTOP);
        } else if (!strcmp(argv[1], "set_init"))
        {
          or_set_safety_mode(OR_SAFETY_INIT);
        } else if (!strcmp(argv[1], "set_brake"))
        {
          or_set_safety_mode(OR_SAFETY_BRAKE);
        } else if (!strcmp(argv[1], "clear_all"))
        {
          for (int i = 0; i < ERR_COUNT; ++i)
          {
            or_active_errors[i] = false;
          }
          or_comm.state->word &= ~OR_STATE_ERR_MSK;
        } else if (!strcmp(argv[1], "clear_history"))
        {
          for (int i = 0; i < ERR_COUNT; ++i)
          {
            or_error_triggered[i] = false;
          }
        } else if (!strcmp(argv[1], "release"))
        {
          mc_interface_release_motor();
        } else if (!strcmp(argv[1], "print"))
        {
          switch(or_comm.state->word & OR_SAFETY_MSK)
          {
            case OR_SAFETY_INIT:
            {
              commands_printf("Safety state: INIT");
              break;
            }
            case OR_SAFETY_IDLE:
            {
              commands_printf("Safety state: IDLE");
              break;
            }
            case OR_SAFETY_ENABLE:
            {
              commands_printf("Safety state: ENABLE");
              break;
            }
            case OR_SAFETY_HOLD:
            {
              commands_printf("Safety state: HOLD");
              break;
            }
            case OR_SAFETY_BRAKE:
            {
              commands_printf("Safety state: BRAKE");
              break;
            }
            case OR_SAFETY_ESTOP:
            {
              commands_printf("Safety state: ESTOP");
              break;
            }
          }

          switch(or_comm.ctrl->word & OR_CTRL_MODE_MSK)
          {
            case OR_CTRL_MODE_POS:
            {
              commands_printf("Control mode: POS");
              break;
            }
            case OR_CTRL_MODE_VEL:
            {
              commands_printf("Control mode: VEL");
              break;
            }
            case OR_CTRL_MODE_TRQ:
            {
              commands_printf("Control mode: TRQ");
              break;
            }
            case OR_CTRL_MODE_IMP:
            {
              commands_printf("Control mode: IMP - impedance");
              break;
            }
            case OR_CTRL_MODE_CST:
            {
              commands_printf("Control mode: CST - custom");
              break;
            }
            case OR_CTRL_MODE_OFF:
            {
              commands_printf("Control mode: OFF");
              break;
            }
          }

          commands_printf("Active errors:");

          for (int i = 0; i < ERR_COUNT; i++) {
              if (or_active_errors[i]) {
                  commands_printf(" - %s (%s)", or_error_messages[i], or_error_level_txt[or_get_error_severity(i)]);
              }
          }

          commands_printf("Control word: 0x%04X", or_comm.ctrl->word);
          commands_printf("Status word : 0x%04X", or_comm.state->word);
          commands_printf("Auto clear errors: %s", or_conf.auto_clear_errors ? "true" : "false");
          commands_printf("System time (s): % 5d",  ST2S(chVTGetSystemTimeX()));
        } else {
            commands_printf("X.");
        }
    }
}