#include "app_orthopus.h"

#define MAX_CONFIG_U32_SIZE 32

static bool orthopus_config_load(orthopus_config_t* cfg)
{
  const uint8_t sz = sizeof(orthopus_config_t)/4;
  //commands_printf("Cfg: Loading %d dwords from HW EEPROM",sz);
  if(sz > MAX_CONFIG_U32_SIZE)
    return false;
  uint8_t addr=0;
  for(addr=0;addr<sz;addr++)
  {
    eeprom_var v;
    if(conf_general_read_eeprom_var_hw(&v,addr))
    {
      //commands_printf("Cfg: Loading data '0x%04X/% 5d/% 5.3f' from addr 0x%02X",v.as_u32,v.as_i32,(double)v.as_float,addr);
      *(uint32_t*)(cfg+addr) = v.as_u32;
    }
  }
  return true;
}

static bool orthopus_config_save(const orthopus_config_t* cfg)
{
  const uint8_t sz = sizeof(orthopus_config_t)/4;
  //commands_printf("Cfg: Saving %d dwords to HW EEPROM",sz);
  if(sz > MAX_CONFIG_U32_SIZE)
    return false;
  uint8_t addr=0;
  for(addr=0;addr<sz;addr++)
  {
    eeprom_var v; v.as_u32 = *(uint32_t*)(cfg+addr);
    if(!conf_general_store_eeprom_var_hw(&v,addr))
    {
      //commands_printf("Cfg: Failed to save data '0x%04X/% 5d/% 5.3f' at addr 0x%02X",v.as_u32,v.as_i32,(double)v.as_float,addr);
      return false;
    }
  }
  return true;
}

float orthopus_read_encoder(void)
{
  return enc_as504x_read_angle(&encoder_cfg_as504x)-orthopus_config.encoder_offset;
}

float orthopus_read_encoder_raw(void)
{
  return enc_as504x_read_angle(&encoder_cfg_as504x);
}

float orthopus_read_encoder_filtered(void)
{
  //return enc_as504x_read_angle(&encoder_cfg_as504x);
  return app_orthopus_get_enc_pos_filtered();
}

float orthopus_read_pos_multiturn(void)
{
  return app_orthopus_get_pos_multiturn();
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
    v = enc_as504x_read_angle(&encoder_cfg_as504x);
  orthopus_config.encoder_offset = v;
  orthopus_set_joint_offset(0,false);
  return v;
}
