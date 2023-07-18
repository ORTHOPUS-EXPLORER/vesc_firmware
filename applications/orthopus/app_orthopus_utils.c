#include "app_orthopus.h"

#define MAX_CONFIG_U32_SIZE 32

static bool orthopus_config_load(orthopus_config_t* cfg)
{
  const uint8_t sz = sizeof(orthopus_config_t)/4;
  commands_printf("Cfg: Loading %d dwords from HW EEPROM",sz);
  if(sz > MAX_CONFIG_U32_SIZE)
    return false;
  uint8_t addr=0;
  for(addr=0;addr<sz;addr++)
  {
    eeprom_var v;
    if(conf_general_read_eeprom_var_hw(&v,addr))
    {
      uint32_t* raddr = (uint32_t*)((uint8_t*)(cfg)+addr*4);
      commands_printf("Cfg: Loading data from EEPROM 0x%02X to RAM 0x%08p: '0x%04X/% 5d/% 5.3f'",addr,raddr,v.as_u32,v.as_i32,(double)v.as_float);
      *raddr = v.as_u32;
    }
  }
  return true;
}

static bool orthopus_config_save(const orthopus_config_t* cfg)
{
  const uint8_t sz = sizeof(orthopus_config_t)/4;
  commands_printf("Cfg: Saving %d dwords to HW EEPROM",sz);
  if(sz > MAX_CONFIG_U32_SIZE)
    return false;
  uint8_t addr=0;
  for(addr=0;addr<sz;addr++)
  {
    uint32_t* raddr = (uint32_t*)((uint8_t*)(cfg)+addr*4);
    eeprom_var v; v.as_u32 = *raddr;
    if(!conf_general_store_eeprom_var_hw(&v,addr))
    {
      commands_printf("Cfg: Failed to save data from RAM 0x%08p to EEPROM 0x%02X: '0x%04X/% 5d/% 5.3f'",raddr,addr,v.as_u32,v.as_i32,(double)v.as_float);
      return false;
    }
  }
  return true;
}

void orthopus_config_reset(orthopus_config_t* cfg)
{
  cfg->encoder_offset                  = 0.0;
  cfg->encoder_filter_anglestep        = 0.25;
  cfg->encoder_filter_enable           = true; // keep enabled or move encoder filtered multiturn angle estimation
  cfg->encoder_filter_plot_enable      = false;
  cfg->limits_enable                   = true;
  cfg->orthopus_config_set             = false;
  cfg->limits_pos_max                  = 90.0;
  cfg->limits_pos_min                  = -90.0;
  cfg->angle_division                  = 700;
  cfg->limits_reach_angle              = 15;
  cfg->limits_reach_speed              = 2;
  cfg->encoder_filter_error_gain       = 1;
  cfg->rate_hz                         = 2000;
  cfg->perf_compensateexectime         = true;
  cfg->Torquegain                      = 34.8;
  cfg->limits_kp                       = 0.1;
  cfg->limits_kd                       = 5.0;
  cfg->limits_powp                     = 6;
  cfg->limits_powd                     = 1;
  cfg->limits_damp_reachangle          = 7;
  cfg->ctrl_stiffness                  = 0;
  cfg->ctrl_damping                    = 0;
  cfg->deadzone                        = true;
  cfg->a                               = 1;
  cfg->torque_filter_const             = 0.1;
  cfg->deadzone                        = true;
  cfg->ctrl_kd                         = 0;
  cfg->ctrl_kd_filter                  = 1;
  cfg->max_enc_diff                    = 2;
}

float orthopus_read_encoder(void)
{
  return orthopus_read_encoder_raw()-orthopus_config.encoder_offset;
}

float orthopus_read_encoder_raw(void)
{
  if(!orthopus_thread_running)
    enc_as504x_routine(&encoder_cfg_as504x);
  return encoder_cfg_as504x.state.last_enc_angle;
}

float orthopus_set_joint_offset(float v, bool use_v)
{
  if(!use_v)
  {
    v = 0.0; // Reset
    size_t i = 0;
    for(i=0;i<3;i++)
    {
      chThdSleepMilliseconds(1);
      v += orthopus_read_encoder()/3;
    }
  }

  mc_interface_update_pid_pos_offset(v, false);
  return v;
}

float orthopus_set_encoder_offset(float v, bool use_v)
{
  if(!use_v)
    v = orthopus_read_encoder_raw();
  orthopus_state.turn_now = 0;
  orthopus_state.enc_turn = 0;
  orthopus_config.encoder_offset = v;
  orthopus_set_joint_offset(0,false);
  if ( mc_interface_get_pid_pos_now() > 180)
  {
    orthopus_state.turn_now = -1;
  }
  if (orthopus_read_encoder() > 180)
  {
    orthopus_state.enc_turn = -1;
  }
  return v;
}
