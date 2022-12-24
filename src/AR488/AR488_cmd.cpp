
#include "AR488_Config.h"
#include "AR488_GPIBbus.h"
#include "AR488_ComPorts.h"
#include "AR488_Eeprom.h"
#include "AR488_cmd.h"

struct cmdRec cmdHidx [] = { 
  { "addr",        CMD_DEV | CMD_CONTROLLER, addr_h      }, 
  { "allspoll",              CMD_CONTROLLER, (void(*)(char*)) aspoll_h  },
  { "auto",                  CMD_CONTROLLER, amode_h     },
  { "clr",                   CMD_CONTROLLER, (void(*)(char*)) clr_h     },
  { "dcl",                   CMD_CONTROLLER, (void(*)(char*)) dcl_h     },
  { "default",     CMD_DEV | CMD_CONTROLLER, (void(*)(char*)) default_h },
  { "eoi",         CMD_DEV | CMD_CONTROLLER, eoi_h       },
  { "eor",         CMD_DEV | CMD_CONTROLLER, eor_h       },
  { "eos",         CMD_DEV | CMD_CONTROLLER, eos_h       },
  { "eot_char",    CMD_DEV | CMD_CONTROLLER, eot_char_h  },
  { "eot_enable",  CMD_DEV | CMD_CONTROLLER, eot_en_h    },
  { "help",        CMD_DEV | CMD_CONTROLLER, help_h      },
  { "ifc",                   CMD_CONTROLLER, (void(*)(char*)) ifc_h     },
  { "id",          CMD_DEV | CMD_CONTROLLER, id_h        },
  { "idn",         CMD_DEV | CMD_CONTROLLER, idn_h       },
  { "llo",                   CMD_CONTROLLER, llo_h       },
  { "loc",                   CMD_CONTROLLER, loc_h       },
  { "lon",         CMD_DEV                 , lon_h       },
  { "macro",                 CMD_CONTROLLER, macro_h     },
  { "mla",                   CMD_CONTROLLER, (void(*)(char*)) sendmla_h },
  { "mode" ,       CMD_DEV | CMD_CONTROLLER, cmode_h     },
  { "msa",                   CMD_CONTROLLER, sendmsa_h   },
  { "mta",                   CMD_CONTROLLER, (void(*)(char*)) sendmta_h },
  { "ppoll",                 CMD_CONTROLLER, (void(*)(char*)) ppoll_h   },
  { "prom",        CMD_DEV                 , prom_h      },
  { "read",                  CMD_CONTROLLER, read_h      },
  { "read_tmo_ms",           CMD_CONTROLLER, rtmo_h      },
  { "ren",                   CMD_CONTROLLER, ren_h       },
  { "repeat",                CMD_CONTROLLER, repeat_h    },
  { "rst",         CMD_DEV | CMD_CONTROLLER, (void(*)(char*)) rst_h     },
  { "trg",                   CMD_CONTROLLER, trg_h       },
  { "savecfg",     CMD_DEV | CMD_CONTROLLER, (void(*)(char*)) save_h    },
  { "setvstr",     CMD_DEV | CMD_CONTROLLER, setvstr_h   },
  { "spoll",                 CMD_CONTROLLER, spoll_h     },
  { "srq",                   CMD_CONTROLLER, (void(*)(char*)) srq_h     },
  { "srqauto",               CMD_CONTROLLER, srqa_h      },
  { "status",      CMD_DEV                 , stat_h      },
  { "ton",         CMD_DEV                 , ton_h       },
  { "unl",                   CMD_CONTROLLER, (void(*)(char*)) unlisten_h  },
  { "unt",                   CMD_CONTROLLER, (void(*)(char*)) untalk_h    },
  { "ver",         CMD_DEV | CMD_CONTROLLER, ver_h       },
  { "verbose",     CMD_DEV | CMD_CONTROLLER, (void(*)(char*)) verb_h    },
  { "xdiag",       CMD_DEV | CMD_CONTROLLER, xdiag_h     },
  { (const char *)0, 0, 0 }
};
