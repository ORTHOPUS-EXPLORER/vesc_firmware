#include "app_orthopus.h"
#include "mc_interface.h"

bool or_config_load(orthopus_config_t* cfg)
{
  const uint8_t sz = sizeof(orthopus_config_t)/4;
  commands_printf("Cfg: Loading %d dwords from HW EEPROM",sz);
  if(sz > MAX_CONFIG_U32_SIZE)
    return false;
  uint8_t addr=0;
  for(addr=0;addr<sz;addr++)
  {
    eeprom_var v;
    if(conf_general_read_eeprom_var_custom(&v,addr))
    {
      uint32_t* raddr = (uint32_t*)((uint8_t*)(cfg)+addr*4);
      // FIXME: Remove debug
      commands_printf("Cfg: Loading data from EEPROM 0x%02X to RAM 0x%08p: '0x%08X/% 11d/% 11.5f'",addr,raddr,v.as_u32,v.as_i32,(double)v.as_float);
      *raddr = v.as_u32;
    }
  }
  if(cfg->signature != ORTHOPUS_CONFIG_T_SIGNATURE)
  {
    commands_printf("Cfg: Stored signature does not match the one compiled in this firmware: 0x%08X/0x%08X. You may want to start fresh and reset", cfg->signature, ORTHOPUS_CONFIG_T_SIGNATURE);
    return false;
  }
  return true;
}

bool or_config_set(orthopus_config_t* cfg, const uint8_t* buffer)
{
  if(buffer)
  {
    if(!orthopus_confparser_deserialize_orthopus_config_t(buffer, cfg))
      return false;
  }
  else
    orthopus_confparser_set_defaults_orthopus_config_t(cfg);

  cfg->signature = ORTHOPUS_CONFIG_T_SIGNATURE;
  return true;
}

bool or_config_save(const orthopus_config_t* cfg)
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
    // FIXME: Remove debug
    commands_printf("Cfg: Saving data from RAM 0x%08p to EEPROM 0x%02X: '0x%08X/% 11d/% 11.5f'",raddr, addr, v.as_u32, v.as_i32, (double)v.as_float);
    if(!conf_general_store_eeprom_var_custom(&v,addr))
    {
      commands_printf("Cfg: Failed to save data from RAM 0x%08p to EEPROM 0x%02X: '0x%08X/% 11d/% 11.5f'",raddr, addr, v.as_u32, v.as_i32, (double)v.as_float);
      return false;
    }
  }
  return true;
}

float or_read_encoder(void)
{
  float raw_pos = or_read_encoder_raw();
  float offset = or_conf.encoder_offset;
  
  return raw_pos - offset;
}

float or_read_encoder_raw(void)
{
  if(!orthopus_thread_running)
    enc_as504x_routine(&encoder_cfg_as504x);
  
  float raw_angle = encoder_cfg_as504x.state.last_enc_angle;
  
  // Apply direction correction if motor direction is inverted
  const volatile mc_configuration *mcconf = mc_interface_get_configuration();
  if (mcconf->m_invert_direction) {
    raw_angle = -raw_angle;
  }
  
  return raw_angle;
}

float or_set_joint_offset(float v, bool use_v)
{
  if(!use_v)
  {
    v = 0.0; // Reset
    size_t i = 0;
    for(i=0;i<3;i++)
    {
      chThdSleepMilliseconds(1);
      v += or_read_encoder()/3;
    }
  }

  mc_interface_update_pid_pos_offset(v, false);
  return v;
}

float or_set_encoder_offset(float v, bool use_v)
{
  if(!use_v)
    v = or_read_encoder_raw();
    
  or_state.turn_now = 0;
  or_state.enc_turn = 0;
  or_conf.encoder_offset = v;
  or_set_joint_offset(0,false);
  if ( mc_interface_get_pid_pos_now() > 180)
  {
    or_state.turn_now = -1;
    commands_printf("-1 turn sincos");
  }
  or_state.encoders_init = false;
  return v;
}
