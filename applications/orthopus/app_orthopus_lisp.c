#include "app_orthopus.h"

static lbm_value orthopus_lisp_read_encoder(lbm_value *args, lbm_uint argn);
static lbm_value orthopus_lisp_read_encoder_raw(lbm_value *args, lbm_uint argn);
static lbm_value orthopus_lisp_read_encoder_filtered(lbm_value *args, lbm_uint argn);
static lbm_value orthopus_lisp_read_pos_multiturn(lbm_value *args, lbm_uint argn);
static lbm_value orthopus_lisp_read_speed(lbm_value *args, lbm_uint argn);
static lbm_value orthopus_lisp_read_torque(lbm_value *args, lbm_uint argn);
static lbm_value orthopus_lisp_offset(lbm_value *args, lbm_uint argn);
static lbm_value orthopus_lisp_config(lbm_value *args, lbm_uint argn);

static lbm_uint orthopus_lisp_s0
              , orthopus_lisp_s1;

static lbm_value orthopus_ca_sym;
static unsigned char orthopus_ca[3] = { 0x01, 0x42, 0x03 };
static lbm_uint orthopus_fv_sym,orthopus_fc_sym;

static lbm_value orthopus_lisp_test_symbols(lbm_value *args, lbm_uint argn);

void orthopus_init_lisp(void)
{
    // in REPL, test with: (orthopus-test-syms 'orthopus0)    => ok
    //                     (orthopus-test-syms 'orthopus1)    => ok
    //                     (orthopus-test-syms 'not_a_symbol) => eval_error
  lbm_add_variable_symbol_const("orthopus0", &orthopus_lisp_s0);
  lbm_add_variable_symbol_const("orthopus1", &orthopus_lisp_s1);
  lbm_add_extension("orthopus-test-syms", orthopus_lisp_test_symbols);

  // Share array
  // Access from lisp: Read:  (print (bufget-u8 orthopus-ca 1))
  //                   Write: (bufset-u8 orthopus-ca 1 65)
  if(lbm_share_array(&orthopus_ca_sym,(char*)orthopus_ca, LBM_TYPE_CHAR, 3))
    lbm_define("orthopus-ca",orthopus_ca_sym);
  // bufset only exists for -(i/u)(8/16/32), so...
  // not really sure about the compatibility of types of orthopus_ca and the LBM_TYPE_***
  // unsigned_char and LBM_TYPE_CHAR is the obvious/easiest test.

  // Set LISP variable-symbol from C
  // Access from LISP, in REPL: Read  (print   #orthopus-fv)
  //                            Write (define  #orthopus-fv 15.90)
  //                               or (set-var #orthopus-fc 15.91)
  lbm_define("#orthopus-fv",lbm_enc_float(12.34));         // Define variable symbol, # is important
  lbm_get_symbol_by_name("#orthopus-fv",&orthopus_fv_sym); // Save index

  // Set LISP symbol from C
  // Access from LISP, in REPL: Read  (print    orthopus-fc)
  //                            Write (define   orthopus-fc 15.92)
  //                               or (set-var 'orthopus-fc 15.93)
  lbm_define("orthopus-fc",lbm_enc_float(43.21));         // Define symbol
  lbm_get_symbol_by_name("orthopus-fc",&orthopus_fc_sym); // Save index

  // Not sure on the differences between variable-symbols and symbols
  // From: https://github.com/svenssonjoel/lispBM/blob/master/doc/lbmref.md at the beginning, in the "Symbols" section
  // I guess "variable-symbols" are somewhat faster to access: "start with a #, for fast-lookup variables"
  // From what I have found, variable-symbols are much easier to access from C. YMMV...

    // in REPL, test with: (print (orthopus-read-encoder))
  lbm_add_extension("orthopus-read-encoder", orthopus_lisp_read_encoder);
  lbm_add_extension("orthopus-read-encoder-raw", orthopus_lisp_read_encoder_raw);
  lbm_add_extension("orthopus-read-encoder-filt", orthopus_lisp_read_encoder_filtered);
  lbm_add_extension("orthopus-read-pos-multiturn", orthopus_lisp_read_pos_multiturn);
  lbm_add_extension("orthopus-read-speed", orthopus_lisp_read_speed);
  lbm_add_extension("orthopus-read-torque", orthopus_lisp_read_torque);
  // in REPL, test with: (orthopus-offset "encoder") or (orthopus-init-offset "encoder" 45)
  lbm_add_extension("orthopus-offset", orthopus_lisp_offset);
  // in REPL, test with: (orthopus-config) or (orthopus-config "print/load/save")
  lbm_add_extension("orthopus-config", orthopus_lisp_config);
}


static lbm_value orthopus_lisp_read_encoder(lbm_value *args, lbm_uint argn)
{
	(void)args; (void)argn;
	return lbm_enc_float(orthopus_read_encoder());
}

static lbm_value orthopus_lisp_read_encoder_raw(lbm_value *args, lbm_uint argn)
{
	(void)args; (void)argn;
	return lbm_enc_float(orthopus_read_encoder_raw());
}

static lbm_value orthopus_lisp_read_encoder_filtered(lbm_value *args, lbm_uint argn)
{
	(void)args; (void)argn;
	return lbm_enc_float(orthopus_state.enc_pos_filter);
}

static lbm_value orthopus_lisp_read_pos_multiturn(lbm_value *args, lbm_uint argn)
{
	(void)args; (void)argn;
	//return lbm_enc_float(orthopus_read_pos_multiturn());
  return lbm_enc_float(orthopus_state.pos_multiturn_now);
}

static lbm_value orthopus_lisp_read_speed(lbm_value *args, lbm_uint argn)
{
	(void)args; (void)argn;
	//return lbm_enc_float(orthopus_read_pos_multiturn());
  return lbm_enc_float(orthopus_state.speed_now);
}

static lbm_value orthopus_lisp_read_torque(lbm_value *args, lbm_uint argn)
{
	(void)args; (void)argn;
  return lbm_enc_float(orthopus_state.Torque);
}

static lbm_value orthopus_lisp_offset(lbm_value *args, lbm_uint argn)
{
  //LBM_CHECK_ARGN_NUMBER(1);
  if(argn > 2)
  {
    lbm_set_error_reason("Invalid arguments. Usage: orthopus-offset <\"joint\"|\"encoder\"> [v]");
    return ENC_SYM_EERROR;
  }

  float v = 0;
  if(argn == 2)
    v = lbm_dec_as_float(args[1]);
  const char* s = lbm_dec_str(args[0]);
  if(!strcmp(s,"joint"))
  {
    v = orthopus_set_joint_offset(v, argn == 2);
    commands_printf_lisp("Init Joint (ie. PosPID) offset: % 5.3f", (double)v);
  }
  else if(!strcmp(s,"encoder"))
  {
    v = orthopus_set_encoder_offset(v, argn == 2);
    commands_printf_lisp("Init Encoder offset: % 5.3f", (double)v);
  }

  return ENC_SYM_TRUE;
}

static lbm_value orthopus_lisp_config(lbm_value *args, lbm_uint argn)
{
  //LBM_CHECK_ARGN_NUMBER(1);
  if(argn == 0)
  {
    lbm_set_error_reason("Invalid arguments. Usage: orthopus-config <\"print\"|\"load\"|\"save\">");
    return ENC_SYM_EERROR;
  }

  const char* s = lbm_dec_str(args[0]);
  if(!strcmp(s,"print"))
  {
    commands_printf_lisp("Encoder offset: % 5.3f", (double)orthopus_config.encoder_offset);
  }
  else if(!strcmp(s,"load"))
  {
    if(orthopus_config_load(&orthopus_config))
      commands_printf_lisp("Orthopus config loaded from EEPROM");
    else
      commands_printf_lisp("Orthopus config load failed =/");
  }
  else if(!strcmp(s,"save"))
  {
    if(orthopus_config_save(&orthopus_config))
      commands_printf("Orthopus config saved to EEPROM");
    else
      commands_printf("Orthopus config save failed =/");
  }
  return ENC_SYM_TRUE;
}

static lbm_value orthopus_lisp_test_symbols(lbm_value *args, lbm_uint argn)
{
  if (argn != 1)
		return ENC_SYM_EERROR;


  // Read array
  commands_printf_lisp("orthopus-ca[1] 0x%02X/%d/'%c'", orthopus_ca[1],orthopus_ca[1],orthopus_ca[1]);
  // Read variable-symbol
  commands_printf_lisp("orthopus-fv %f", (double)lbm_dec_as_float(lbm_get_var(orthopus_fv_sym)));
  // Read symbol
  commands_printf_lisp("orthopus-fc %f", (double)lbm_dec_as_float(lbm_env_lookup(lbm_enc_sym(orthopus_fc_sym), lbm_get_env())));

	lbm_uint name = lbm_dec_sym(args[0]);
	if (name == orthopus_lisp_s0) {
		commands_printf_lisp("Found Orthopus symbol 0");
    // Update array
    orthopus_ca[1] = 0x43;
    // Update variable-symbol
    lbm_set_var(orthopus_fv_sym, lbm_enc_float(45.67));
    // Update symbol
    *lbm_get_env_ptr() = lbm_env_modify_binding(lbm_get_env(), lbm_enc_sym(orthopus_fc_sym), lbm_enc_float(76.54));
	} else if (name == orthopus_lisp_s1) {
		commands_printf_lisp("Found Orthopus symbol 1");
	} else {
		return ENC_SYM_EERROR;
	}

	return ENC_SYM_TRUE;
}
