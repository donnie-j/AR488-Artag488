#ifndef __AR488_CMD__
#define __AR488_CMD__

struct cmdRec { 
  const char* token; 
  int opmode;
  void (*handler)(char *);
};

extern struct cmdRec cmdHidx [];

#define CMD_DEV 1
#define CMD_CONTROLLER 2

void errBadCmd();
bool isCmd(char *buffr);
bool isIdnQuery(char *buffr);
bool isRead(char *buffr);
bool notInRange(char *param, uint16_t lowl, uint16_t higl, uint16_t &rval);

void addr_h(char *params);
void rtmo_h(char *params);
void eos_h(char *params);
void eoi_h(char *params);
void cmode_h(char *params);
void eot_en_h(char *params);
void eot_char_h(char *params);
void amode_h(char *params);
void ver_h(char *params);
void read_h(char *params);
void clr_h();
void llo_h(char *params);
void loc_h(char *params);
void ifc_h();
void trg_h(char *params);
void rst_h();
void spoll_h(char *params);
void srq_h();
void stat_h(char *params);
void save_h();
void lon_h(char *params);
void help_h(char *params);
void aspoll_h();
void dcl_h();
void default_h();
void eor_h(char *params);
void ppoll_h();
void ren_h(char *params);
void verb_h();
void setvstr_h(char *params);
void prom_h(char *params);
void ton_h(char *params);
void srqa_h(char *params);
void repeat_h(char *params);
void macro_h(char *params);
void xdiag_h(char *params);
void id_h(char *params);
void idn_h(char * params);
void sendmla_h();
void sendmta_h();
void sendmsa_h(char *params);
void unlisten_h();
void untalk_h();

void attnRequired();
void execGpibCmd(uint8_t gpibcmd);
void device_listen_h();
void device_talk_h();
void device_sdc_h();
void device_spd_h();
void device_spe_h();
bool device_unl_h();
bool device_unt_h();
void lonMode();
void tonMode();

#endif /* __AR488_CMD_H__ */
