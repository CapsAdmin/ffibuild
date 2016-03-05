local ffi = require("ffi")
ffi.cdef([[struct __mbstate_t {int __count;union {unsigned int __wch;char __wchb[4];}__value;};
struct _G_fpos_t {long __pos;struct __mbstate_t __state;};
struct _IO_FILE {};
struct screen {};
struct ldat {};
struct _win_st {short _cury;short _curx;short _maxy;short _maxx;short _begy;short _begx;short _flags;unsigned int _attrs;unsigned int _bkgd;_Bool _notimeout;_Bool _clear;_Bool _leaveok;_Bool _scroll;_Bool _idlok;_Bool _idcok;_Bool _immed;_Bool _sync;_Bool _use_keypad;int _delay;struct ldat*_line;short _regtop;short _regbottom;int _parx;int _pary;struct _win_st*_parent;struct {short _pad_y;short _pad_x;short _pad_top;short _pad_left;short _pad_bottom;short _pad_right;}_pad;short _yoffset;};
struct MEVENT {short id;int x;int y;int z;unsigned int bstate;};
int(ripoffline_sp)(SCREEN*,int,int(*)(struct _win_st*,int);
int(fseek)(struct _IO_FILE*,long,int);
int(wclrtoeol)(struct _win_st*);
int(waddstr)(struct _win_st*,const char*);
int(intrflush_sp)(struct screen*,struct _win_st*,_Bool);
_Bool(is_nodelay)(const struct _win_st*);
int(mvgetnstr)(int,int,char*,int);
int(endwin_sp)(struct screen*);
int(wgetnstr)(struct _win_st*,char*,int);
int(scrollok)(struct _win_st*,_Bool);
int(getmaxy)(const struct _win_st*);
char*(termname)();
int(putc)(int,struct _IO_FILE*);
int(nodelay)(struct _win_st*,_Bool);
int(inchnstr)(unsigned int*,int);
int(vidputs)(unsigned int,int(*unknown_2)(int));
int(fgetc)(struct _IO_FILE*);
int(mvaddstr)(int,int,const char*);
int(scr_init)(const char*);
int(meta)(struct _win_st*,_Bool);
int(getmouse)(struct MEVENT*);
unsigned int(getbkgd)(struct _win_st*);
int(whline)(struct _win_st*,unsigned int,int);
char*(longname_sp)(struct screen*);
int(addchnstr)(const unsigned int*,int);
int(mvwinchnstr)(struct _win_st*,int,int,unsigned int*,int);
struct _win_st*(subpad)(struct _win_st*,int,int,int,int);
int(use_default_colors_sp)(struct screen*);
int(ftrylockfile)(struct _IO_FILE*);
int(wtouchln)(struct _win_st*,int,int,int);
int(mvgetch)(int,int);
int(slk_clear_sp)(struct screen*);
void(_tracef)(const char*,...);
unsigned int(inch)();
unsigned int(slk_attr_sp)(struct screen*);
int(standend)();
void(noqiflush)();
_Bool(wenclose)(const struct _win_st*,int,int);
int(border)(unsigned int,unsigned int,unsigned int,unsigned int,unsigned int,unsigned int,unsigned int,unsigned int);
int(noraw)();
int(nl_sp)(struct screen*);
unsigned int(mousemask)(unsigned int,unsigned int*);
int(slk_attr_off)(const unsigned int,void*);
struct _IO_FILE*(open_memstream)(char**,unsigned long*);
int(vsnprintf)(char*,unsigned long,const char*,__builtin_va_list);
int(mvwdelch)(struct _win_st*,int,int);
int(vscanf)(const char*,__builtin_va_list);
int(getc_unlocked)(struct _IO_FILE*);
int(slk_refresh_sp)(struct screen*);
int(ungetmouse)(struct MEVENT*);
char*(_tracechtype2)(int,unsigned int);
int(mvwhline)(struct _win_st*,int,int,unsigned int,int);
long(__io_read_fn)(void*,char*,unsigned long);
int(mvwinsstr)(struct _win_st*,int,int,const char*);
int(slk_touch_sp)(struct screen*);
int(mvderwin)(struct _win_st*,int,int);
int(mvwaddch)(struct _win_st*,int,int,const unsigned int);
int(putw)(int,struct _IO_FILE*);
int(mvgetstr)(int,int,char*);
int(feof)(struct _IO_FILE*);
int(getch)();
int(flushinp)();
int(savetty_sp)(struct screen*);
int(color_set)(short,void*);
int(mvwgetstr)(struct _win_st*,int,int,char*);
long(_IO_seekpos)(struct _IO_FILE*,long,int);
int(sprintf)(char*,const char*,...);
int(tigetflag)(const char*);
int(vwscanw)(struct _win_st*,const char*,__builtin_va_list);
int(addnstr)(const char*,int);
char*(_tracechar)(int);
int(wgetscrreg)(const struct _win_st*,int*,int*);
struct screen*(set_term)(struct screen*);
int(napms_sp)(struct screen*,int);
int(slk_noutrefresh)();
int(mvwgetch)(struct _win_st*,int,int);
int(mvinsstr)(int,int,const char*);
char*(tparm)(const char*,...);
int(mvwscanw)(struct _win_st*,int,int,const char*,...);
int(getmouse_sp)(struct screen*,struct MEVENT*);
void(_IO_funlockfile)(struct _IO_FILE*);
int(vfprintf)(struct _IO_FILE*,const char*,__builtin_va_list);
int(mvdelch)(int,int);
char*(slk_label_sp)(struct screen*,int);
int(wechochar)(struct _win_st*,const unsigned int);
int(copywin)(const struct _win_st*,struct _win_st*,int,int,int,int,int,int,int);
int(wstandend)(struct _win_st*);
int(prefresh)(struct _win_st*,int,int,int,int,int,int);
int(use_legacy_coding_sp)(struct screen*,int);
int(mvwinstr)(struct _win_st*,int,int,char*);
int(reset_shell_mode)();
int(mvwgetnstr)(struct _win_st*,int,int,char*,int);
int(mvwinsnstr)(struct _win_st*,int,int,const char*,int);
int(wsetscrreg)(struct _win_st*,int,int);
char*(_traceattr2)(int,unsigned int);
_Bool(mouse_trafo)(int*,int*,_Bool);
void(qiflush_sp)(struct screen*);
int(delay_output)(int);
char*(tempnam)(const char*,const char*);
int(standout)();
const char*(keyname_sp)(struct screen*,int);
void(use_tioctl_sp)(struct screen*,_Bool);
int(inchstr)(unsigned int*);
int(mvwaddnstr)(struct _win_st*,int,int,const char*,int);
int(fsetpos)(struct _IO_FILE*,const struct _G_fpos_t*);
int(waddchstr)(struct _win_st*,const unsigned int*);
int(flushinp_sp)(struct screen*);
int(halfdelay)(int);
int(wclrtobot)(struct _win_st*);
int(pnoutrefresh)(struct _win_st*,int,int,int,int,int,int);
_Bool(isendwin)();
void(noqiflush_sp)(struct screen*);
int(scr_init_sp)(struct screen*,const char*);
int(__uflow)(struct _IO_FILE*);
char(killchar_sp)(struct screen*);
int(getmaxx)(const struct _win_st*);
int(addchstr)(const unsigned int*);
void(_IO_flockfile)(struct _IO_FILE*);
int(assume_default_colors_sp)(struct screen*,int,int);
int(leaveok)(struct _win_st*,_Bool);
int(insnstr)(const char*,int);
int(pclose)(struct _IO_FILE*);
int(vsprintf)(char*,const char*,__builtin_va_list);
int(bkgd)(unsigned int);
int(slk_attron_sp)(struct screen*,const unsigned int);
struct _IO_FILE*(freopen)(const char*,const char*,struct _IO_FILE*);
int(idlok)(struct _win_st*,_Bool);
int(resizeterm_sp)(struct screen*,int,int);
int(chgat)(int,unsigned int,short,const void*);
int(has_key_sp)(struct screen*,int);
int(slk_attroff_sp)(struct screen*,const unsigned int);
unsigned int(winch)(struct _win_st*);
int(getchar)();
int(use_default_colors)();
const char*(unctrl)(unsigned int);
int(wprintw)(struct _win_st*,const char*,...);
int(beep_sp)(struct screen*);
int(def_shell_mode_sp)(struct screen*);
int(getattrs)(const struct _win_st*);
int(slk_attr_on)(unsigned int,void*);
int(_IO_vfscanf)(struct _IO_FILE*,const char*,__builtin_va_list,int*);
int(fputs)(const char*,struct _IO_FILE*);
int(wbkgd)(struct _win_st*,unsigned int);
int(curs_set_sp)(struct screen*,int);
int(wchgat)(struct _win_st*,int,unsigned int,short,const void*);
unsigned int(mousemask_sp)(struct screen*,unsigned int,unsigned int*);
char*(tmpnam_r)(char*);
unsigned long(fread_unlocked)(void*,unsigned long,unsigned long,struct _IO_FILE*);
int(resize_term_sp)(struct screen*,int,int);
int(instr)(char*);
int(_IO_vfprintf)(struct _IO_FILE*,const char*,__builtin_va_list);
int(winchstr)(struct _win_st*,unsigned int*);
void(idcok)(struct _win_st*,_Bool);
int(vw_scanw)(struct _win_st*,const char*,__builtin_va_list);
int(getstr)(char*);
int(remove)(const char*);
void(use_env_sp)(struct screen*,_Bool);
int(wattr_set)(struct _win_st*,unsigned int,short,void*);
int(_IO_putc)(int,struct _IO_FILE*);
int(slk_attrset_sp)(struct screen*,const unsigned int);
int(getparx)(const struct _win_st*);
unsigned long(fread)(void*,unsigned long,unsigned long,struct _IO_FILE*);
int(slk_attron)(const unsigned int);
char*(tigetstr)(const char*);
int(slk_refresh)();
int(vw_printw)(struct _win_st*,const char*,__builtin_va_list);
int(wattroff)(struct _win_st*,int);
int(clrtobot)();
int(attron)(int);
int(touchline)(struct _win_st*,int,int);
char*(_traceattr)(unsigned int);
int(__overflow)(struct _IO_FILE*,int);
int(raw)();
int(scanw)(const char*,...);
int(nonl)();
int(wscanw)(struct _win_st*,const char*,...);
struct _win_st*(getwin)(struct _IO_FILE*);
int(innstr)(char*,int);
int(raw_sp)(struct screen*);
int(attroff)(int);
int(scrl)(int);
int(resetty_sp)(struct screen*);
int(use_legacy_coding)(int);
int(scr_restore)(const char*);
char*(_tracechtype)(unsigned int);
int(use_window)(struct _win_st*,int(*unknown_2)(struct _win_st*,void*),void*);
int(clear)();
void(bkgdset)(unsigned int);
int(mvaddchnstr)(int,int,const unsigned int*,int);
int(wattrset)(struct _win_st*,int);
int(vfscanf)(struct _IO_FILE*,const char*,__builtin_va_list);
int(keyok_sp)(struct screen*,int,_Bool);
int(mouseinterval_sp)(struct screen*,int);
_Bool(is_keypad)(const struct _win_st*);
void(perror)(const char*);
int(define_key)(const char*,int);
_Bool(is_notimeout)(const struct _win_st*);
void(trace)(const unsigned int);
int(noecho_sp)(struct screen*);
int(use_screen)(struct screen*,int(*unknown_2)(struct screen*,void*),void*);
int(ferror)(struct _IO_FILE*);
int(syncok)(struct _win_st*,_Bool);
_Bool(is_immedok)(const struct _win_st*);
int(addch)(const unsigned int);
_Bool(is_scrollok)(const struct _win_st*);
int(endwin)();
int(set_tabsize_sp)(struct screen*,int);
int(ungetmouse_sp)(struct screen*,struct MEVENT*);
int(mvwin)(struct _win_st*,int,int);
int(mvaddnstr)(int,int,const char*,int);
struct _IO_FILE*(popen)(const char*,const char*);
int(cbreak_sp)(struct screen*);
int(mvcur_sp)(struct screen*,int,int,int,int);
void(clearerr)(struct _IO_FILE*);
int(nl)();
int(ferror_unlocked)(struct _IO_FILE*);
int(slk_restore_sp)(struct screen*);
int(noraw_sp)(struct screen*);
void(wbkgdset)(struct _win_st*,unsigned int);
int(werase)(struct _win_st*);
char(erasechar_sp)(struct screen*);
int(wattr_on)(struct _win_st*,unsigned int,void*);
_Bool(is_leaveok)(const struct _win_st*);
void(delscreen)(struct screen*);
void(nofilter_sp)(struct screen*);
int(delwin)(struct _win_st*);
int(wborder)(struct _win_st*,unsigned int,unsigned int,unsigned int,unsigned int,unsigned int,unsigned int,unsigned int,unsigned int);
int(renameat)(int,const char*,int,const char*);
int(mvinchnstr)(int,int,unsigned int*,int);
int(init_color_sp)(struct screen*,short,short,short,short);
struct _win_st*(derwin)(struct _win_st*,int,int,int,int);
int(def_prog_mode)();
int(mvinstr)(int,int,char*);
char*(ctermid)(char*);
char*(termname_sp)(struct screen*);
int(erase)();
int(has_key)(int);
int(scr_set)(const char*);
int(set_escdelay_sp)(struct screen*,int);
int(putc_unlocked)(int,struct _IO_FILE*);
int(wscrl)(struct _win_st*,int);
int(mvscanw)(int,int,const char*,...);
void(setlinebuf)(struct _IO_FILE*);
_Bool(has_colors_sp)(struct screen*);
int(refresh)();
int(mvaddch)(int,int,const unsigned int);
int(scanf)(const char*,...);
int(slk_touch)();
int(mouseinterval)(int);
long(getline)(char**,unsigned long*,struct _IO_FILE*);
int(mvwchgat)(struct _win_st*,int,int,int,unsigned int,short,const void*);
int(wattr_off)(struct _win_st*,unsigned int,void*);
long(_IO_seekoff)(struct _IO_FILE*,long,int,int);
int(_IO_ftrylockfile)(struct _IO_FILE*);
int(mvinnstr)(int,int,char*,int);
unsigned long(fwrite_unlocked)(const void*,unsigned long,unsigned long,struct _IO_FILE*);
_Bool(has_colors)();
int(_IO_getc)(struct _IO_FILE*);
int(delay_output_sp)(struct screen*,int);
int(wstandout)(struct _win_st*);
int(echo_sp)(struct screen*);
void(qiflush)();
long(ftell)(struct _IO_FILE*);
int(mvwinsch)(struct _win_st*,int,int,unsigned int);
int(set_escdelay)(int);
char*(slk_label)(int);
int(attr_on)(unsigned int,void*);
char*(longname)();
int(doupdate_sp)(struct screen*);
int(sscanf)(const char*,const char*,...);
int(_IO_feof)(struct _IO_FILE*);
int(wresize)(struct _win_st*,int,int);
int(init_color)(short,short,short,short);
int(mvwinchstr)(struct _win_st*,int,int,unsigned int*);
int(def_prog_mode_sp)(struct screen*);
int(deleteln)();
int(dprintf)(int,const char*,...);
int(mvinsnstr)(int,int,const char*,int);
int(winsertln)(struct _win_st*);
void(wtimeout)(struct _win_st*,int);
_Bool(is_linetouched)(struct _win_st*,int);
int(mvinchstr)(int,int,unsigned int*);
char*(tmpnam)(char*);
int(mvhline)(int,int,unsigned int,int);
char*(keybound_sp)(struct screen*,int,int);
long(getdelim)(char**,unsigned long*,int,struct _IO_FILE*);
int(pair_content)(short,short*,short*);
int(mvwaddchnstr)(struct _win_st*,int,int,const unsigned int*,int);
int(assume_default_colors)(int,int);
int(typeahead)(int);
int(delch)();
int(reset_prog_mode_sp)(struct screen*);
void(nofilter)();
_Bool(wmouse_trafo)(const struct _win_st*,int*,int*,_Bool);
int(getbegy)(const struct _win_st*);
int(fgetc_unlocked)(struct _IO_FILE*);
int(mvwaddchstr)(struct _win_st*,int,int,const unsigned int*);
int(fputc)(int,struct _IO_FILE*);
int(getchar_unlocked)();
const char*(keyname)(int);
int(pechochar)(struct _win_st*,const unsigned int);
int(vdprintf)(int,const char*,__builtin_va_list);
struct screen*(newterm_sp)(struct screen*,const char*,struct _IO_FILE*,struct _IO_FILE*);
struct _IO_FILE*(fdopen)(int,const char*);
int(fileno_unlocked)(struct _IO_FILE*);
int(init_pair)(short,short,short);
int(slk_noutrefresh_sp)(struct screen*);
struct screen*(new_prescr)();
int(savetty)();
int(vidputs_sp)(struct screen*,unsigned int,int(*unknown_3)(struct screen*,int));
int(wrefresh)(struct _win_st*);
int(curs_set)(int);
unsigned int(termattrs)();
int(box)(struct _win_st*,unsigned int,unsigned int);
int(__io_seek_fn)(void*,long*,int);
int(setvbuf)(struct _IO_FILE*,char*,int,unsigned long);
int(reset_shell_mode_sp)(struct screen*);
int(halfdelay_sp)(struct screen*,int);
int(wgetdelay)(const struct _win_st*);
char(killchar)();
int(putwin)(struct _win_st*,struct _IO_FILE*);
int(fileno)(struct _IO_FILE*);
int(waddnstr)(struct _win_st*,const char*,int);
int(wattron)(struct _win_st*,int);
int(slk_init_sp)(struct screen*,int);
int(wnoutrefresh)(struct _win_st*);
int(PAIR_NUMBER)(int);
int(winsnstr)(struct _win_st*,const char*,int);
int(wvline)(struct _win_st*,unsigned int,int);
int(doupdate)();
int(addstr)(const char*);
_Bool(has_il)();
int(def_shell_mode)();
int(keypad)(struct _win_st*,_Bool);
void(use_tioctl)(_Bool);
_Bool(is_term_resized)(int,int);
int(getw)(struct _IO_FILE*);
int(slk_init)(int);
struct _win_st*(newwin_sp)(struct screen*,int,int,int,int);
int(_IO_ferror)(struct _IO_FILE*);
int(vsscanf)(const char*,const char*,__builtin_va_list);
int(scr_dump)(const char*);
_Bool(is_term_resized_sp)(struct screen*,int,int);
int(insertln)();
int(nocbreak_sp)(struct screen*);
int(scr_restore_sp)(struct screen*,const char*);
int(getpary)(const struct _win_st*);
int(mcprint_sp)(struct screen*,char*,int);
int(use_extended_names)(_Bool);
int(mcprint)(char*,int);
unsigned long(_IO_sgetn)(struct _IO_FILE*,void*,unsigned long);
int(slk_set)(int,const char*,int);
_Bool(has_mouse_sp)(struct screen*);
int(get_escdelay_sp)(struct screen*);
int(feof_unlocked)(struct _IO_FILE*);
int(noecho)();
int(mvchgat)(int,int,int,unsigned int,short,const void*);
void(funlockfile)(struct _IO_FILE*);
struct _win_st*(newpad)(int,int);
int(fflush_unlocked)(struct _IO_FILE*);
int(move)(int,int);
int(printf)(const char*,...);
int(vwprintw)(struct _win_st*,const char*,__builtin_va_list);
int(putchar)(int);
_Bool(isendwin_sp)(struct screen*);
int(slk_color)(short);
void(wcursyncup)(struct _win_st*);
int(fputc_unlocked)(int,struct _IO_FILE*);
int(__io_close_fn)(void*);
struct _win_st*(initscr)();
int(mvwinnstr)(struct _win_st*,int,int,char*,int);
void(clearerr_unlocked)(struct _IO_FILE*);
int(rename)(const char*,const char*);
int(insdelln)(int);
_Bool(is_syncok)(const struct _win_st*);
int(winstr)(struct _win_st*,char*);
int(color_content)(short,short*,short*,short*);
int(baudrate)();
void(_IO_free_backup_area)(struct _IO_FILE*);
int(clrtoeol)();
int(hline)(unsigned int,int);
int(fgetpos)(struct _IO_FILE*,struct _G_fpos_t*);
void(wsyncup)(struct _win_st*);
int(insch)(unsigned int);
int(get_escdelay)();
long(__io_write_fn)(void*,const char*,unsigned long);
int(mvprintw)(int,int,const char*,...);
int(attr_off)(unsigned int,void*);
int(echo)();
int(putchar_unlocked)(int);
int(color_content_sp)(struct screen*,short,short*,short*,short*);
int(fseeko)(struct _IO_FILE*,long,int);
int(clearok)(struct _win_st*,_Bool);
struct _win_st*(getwin_sp)(struct screen*,struct _IO_FILE*);
void(setbuf)(struct _IO_FILE*,char*);
int(winsch)(struct _win_st*,unsigned int);
int(vline)(unsigned int,int);
int(fclose)(struct _IO_FILE*);
struct _win_st*(newpad_sp)(struct screen*,int,int);
char*(fgets)(char*,int,struct _IO_FILE*);
_Bool(is_idlok)(const struct _win_st*);
int(wdeleteln)(struct _win_st*);
int(fscanf)(struct _IO_FILE*,const char*,...);
int(flash_sp)(struct screen*);
int(snprintf)(char*,unsigned long,const char*,...);
int(napms)(int);
int(slk_attrset)(const unsigned int);
struct _win_st*(newwin)(int,int,int,int);
int(overwrite)(const struct _win_st*,struct _win_st*);
int(flash)();
struct _win_st*(dupwin)(struct _win_st*);
int(attr_set)(unsigned int,short,void*);
int(notimeout)(struct _win_st*,_Bool);
int(keyok)(int,_Bool);
_Bool(has_il_sp)(struct screen*);
long(_IO_padn)(struct _IO_FILE*,int,long);
int(resizeterm)(int,int);
unsigned int(termattrs_sp)(struct screen*);
int(mvaddchstr)(int,int,const unsigned int*);
_Bool(is_idcok)(const struct _win_st*);
int(overlay)(const struct _win_st*,struct _win_st*);
int(reset_prog_mode)();
int(typeahead_sp)(struct screen*,int);
void(filter)();
void(immedok)(struct _win_st*,_Bool);
int(setscrreg)(int,int);
int(mvinsch)(int,int,unsigned int);
int(redrawwin)(struct _win_st*);
const char*(curses_version)();
int(ungetc)(int,struct _IO_FILE*);
struct _IO_FILE*(fmemopen)(void*,unsigned long,const char*);
int(slk_clear)();
unsigned int(mvinch)(int,int);
int(getc)(struct _IO_FILE*);
struct _win_st*(wgetparent)(const struct _win_st*);
void(setbuffer)(struct _IO_FILE*,char*,unsigned long);
int(pair_content_sp)(struct screen*,short,short*,short*);
int(puts)(const char*);
int(getbegx)(const struct _win_st*);
int(beep)();
int(fflush)(struct _IO_FILE*);
long(__getdelim)(char**,unsigned long*,int,struct _IO_FILE*);
int(printw)(const char*,...);
_Bool(is_subwin)(const struct _win_st*);
int(wclear)(struct _win_st*);
unsigned int(slk_attr)();
int(slk_attr_set)(const unsigned int,short,void*);
int(vidattr)(unsigned int);
char(erasechar)();
unsigned int(mvwinch)(struct _win_st*,int,int);
int(scr_set_sp)(struct screen*,const char*);
int(touchwin)(struct _win_st*);
int(mvvline)(int,int,unsigned int,int);
_Bool(is_wintouched)(struct _win_st*);
int(getcurx)(const struct _win_st*);
int(slk_attroff)(const unsigned int);
int(set_tabsize)(int);
int(scroll)(struct _win_st*);
struct screen*(newterm)(const char*,struct _IO_FILE*,struct _IO_FILE*);
int(tigetnum)(const char*);
int(slk_set_sp)(struct screen*,int,const char*,int);
int(waddch)(struct _win_st*,const unsigned int);
int(resize_term)(int,int);
int(fprintf)(struct _IO_FILE*,const char*,...);
int(untouchwin)(struct _win_st*);
void(use_env)(_Bool);
struct _win_st*(subwin)(struct _win_st*,int,int,int,int);
void(rewind)(struct _IO_FILE*);
_Bool(can_change_color)();
_Bool(is_pad)(const struct _win_st*);
int(start_color)();
int(waddchnstr)(struct _win_st*,const unsigned int*,int);
int(COLOR_PAIR)(int);
int(wattr_get)(struct _win_st*,unsigned int*,short*,void*);
int(attr_get)(unsigned int*,short*,void*);
int(mvwprintw)(struct _win_st*,int,int,const char*,...);
int(cbreak)();
int(wdelch)(struct _win_st*);
int(key_defined_sp)(struct screen*,const char*);
char*(tiparm)(const char*,...);
int(winchnstr)(struct _win_st*,unsigned int*,int);
int(winnstr)(struct _win_st*,char*,int);
_Bool(has_ic)();
int(slk_attr_set_sp)(struct screen*,const unsigned int,short,void*);
int(winsdelln)(struct _win_st*,int);
unsigned long(fwrite)(const void*,unsigned long,unsigned long,struct _IO_FILE*);
int(wredrawln)(struct _win_st*,int,int);
void(wsyncdown)(struct _win_st*);
int(resetty)();
int(getcury)(const struct _win_st*);
const char*(unctrl_sp)(struct screen*,unsigned int);
char*(keybound)(int,int);
struct _IO_FILE*(fopen)(const char*,const char*);
int(slk_color_sp)(struct screen*,short);
int(key_defined)(const char*);
int(attrset)(int);
int(_IO_peekc_locked)(struct _IO_FILE*);
int(mvwvline)(struct _win_st*,int,int,unsigned int,int);
struct _IO_FILE*(tmpfile)();
_Bool(is_cleared)(const struct _win_st*);
int(putp)(const char*);
int(echochar)(const unsigned int);
int(insstr)(const char*);
int(baudrate_sp)(struct screen*);
_Bool(can_change_color_sp)(struct screen*);
void(filter_sp)(struct screen*);
int(mvcur)(int,int,int,int);
int(ungetch)(int);
int(winsstr)(struct _win_st*,const char*);
_Bool(has_ic_sp)(struct screen*);
int(init_pair_sp)(struct screen*,short,short,short);
int(wcolor_set)(struct _win_st*,short,void*);
int(__underflow)(struct _IO_FILE*);
void(flockfile)(struct _IO_FILE*);
int(slk_restore)();
int(vprintf)(const char*,__builtin_va_list);
int(nonl_sp)(struct screen*);
int(getnstr)(char*,int);
int(mvwaddstr)(struct _win_st*,int,int,const char*);
int(wgetch)(struct _win_st*);
int(wmove)(struct _win_st*,int,int);
void(timeout)(int);
int(wgetstr)(struct _win_st*,char*);
int(ungetch_sp)(struct screen*,int);
int(intrflush)(struct _win_st*,_Bool);
int(start_color_sp)(struct screen*);
int(vidattr_sp)(struct screen*,unsigned int);
int(define_key_sp)(struct screen*,const char*,int);
int(nocbreak)();
long(ftello)(struct _IO_FILE*);
_Bool(has_mouse)();
]])
local CLIB = ffi.load(_G.FFI_LIB or "ncurses")
local library = {}
library = {
	 = CLIB.,
	fseek = CLIB.fseek,
	wclrtoeol = CLIB.wclrtoeol,
	waddstr = CLIB.waddstr,
	intrflush_sp = CLIB.intrflush_sp,
	is_nodelay = CLIB.is_nodelay,
	mvgetnstr = CLIB.mvgetnstr,
	endwin_sp = CLIB.endwin_sp,
	wgetnstr = CLIB.wgetnstr,
	scrollok = CLIB.scrollok,
	getmaxy = CLIB.getmaxy,
	termname = CLIB.termname,
	putc = CLIB.putc,
	nodelay = CLIB.nodelay,
	inchnstr = CLIB.inchnstr,
	vidputs = CLIB.vidputs,
	fgetc = CLIB.fgetc,
	mvaddstr = CLIB.mvaddstr,
	scr_init = CLIB.scr_init,
	meta = CLIB.meta,
	getmouse = CLIB.getmouse,
	getbkgd = CLIB.getbkgd,
	whline = CLIB.whline,
	longname_sp = CLIB.longname_sp,
	addchnstr = CLIB.addchnstr,
	mvwinchnstr = CLIB.mvwinchnstr,
	subpad = CLIB.subpad,
	use_default_colors_sp = CLIB.use_default_colors_sp,
	ftrylockfile = CLIB.ftrylockfile,
	wtouchln = CLIB.wtouchln,
	mvgetch = CLIB.mvgetch,
	slk_clear_sp = CLIB.slk_clear_sp,
	_tracef = CLIB._tracef,
	inch = CLIB.inch,
	slk_attr_sp = CLIB.slk_attr_sp,
	standend = CLIB.standend,
	noqiflush = CLIB.noqiflush,
	wenclose = CLIB.wenclose,
	border = CLIB.border,
	noraw = CLIB.noraw,
	nl_sp = CLIB.nl_sp,
	mousemask = CLIB.mousemask,
	slk_attr_off = CLIB.slk_attr_off,
	open_memstream = CLIB.open_memstream,
	vsnprintf = CLIB.vsnprintf,
	mvwdelch = CLIB.mvwdelch,
	vscanf = CLIB.vscanf,
	getc_unlocked = CLIB.getc_unlocked,
	slk_refresh_sp = CLIB.slk_refresh_sp,
	ungetmouse = CLIB.ungetmouse,
	_tracechtype2 = CLIB._tracechtype2,
	mvwhline = CLIB.mvwhline,
	__io_read_fn = CLIB.__io_read_fn,
	mvwinsstr = CLIB.mvwinsstr,
	slk_touch_sp = CLIB.slk_touch_sp,
	mvderwin = CLIB.mvderwin,
	mvwaddch = CLIB.mvwaddch,
	putw = CLIB.putw,
	mvgetstr = CLIB.mvgetstr,
	feof = CLIB.feof,
	getch = CLIB.getch,
	flushinp = CLIB.flushinp,
	savetty_sp = CLIB.savetty_sp,
	color_set = CLIB.color_set,
	mvwgetstr = CLIB.mvwgetstr,
	_IO_seekpos = CLIB._IO_seekpos,
	sprintf = CLIB.sprintf,
	tigetflag = CLIB.tigetflag,
	vwscanw = CLIB.vwscanw,
	addnstr = CLIB.addnstr,
	_tracechar = CLIB._tracechar,
	wgetscrreg = CLIB.wgetscrreg,
	set_term = CLIB.set_term,
	napms_sp = CLIB.napms_sp,
	slk_noutrefresh = CLIB.slk_noutrefresh,
	mvwgetch = CLIB.mvwgetch,
	mvinsstr = CLIB.mvinsstr,
	tparm = CLIB.tparm,
	mvwscanw = CLIB.mvwscanw,
	getmouse_sp = CLIB.getmouse_sp,
	_IO_funlockfile = CLIB._IO_funlockfile,
	vfprintf = CLIB.vfprintf,
	mvdelch = CLIB.mvdelch,
	slk_label_sp = CLIB.slk_label_sp,
	wechochar = CLIB.wechochar,
	copywin = CLIB.copywin,
	wstandend = CLIB.wstandend,
	prefresh = CLIB.prefresh,
	use_legacy_coding_sp = CLIB.use_legacy_coding_sp,
	mvwinstr = CLIB.mvwinstr,
	reset_shell_mode = CLIB.reset_shell_mode,
	mvwgetnstr = CLIB.mvwgetnstr,
	mvwinsnstr = CLIB.mvwinsnstr,
	wsetscrreg = CLIB.wsetscrreg,
	_traceattr2 = CLIB._traceattr2,
	mouse_trafo = CLIB.mouse_trafo,
	qiflush_sp = CLIB.qiflush_sp,
	delay_output = CLIB.delay_output,
	tempnam = CLIB.tempnam,
	standout = CLIB.standout,
	keyname_sp = CLIB.keyname_sp,
	use_tioctl_sp = CLIB.use_tioctl_sp,
	inchstr = CLIB.inchstr,
	mvwaddnstr = CLIB.mvwaddnstr,
	fsetpos = CLIB.fsetpos,
	waddchstr = CLIB.waddchstr,
	flushinp_sp = CLIB.flushinp_sp,
	halfdelay = CLIB.halfdelay,
	wclrtobot = CLIB.wclrtobot,
	pnoutrefresh = CLIB.pnoutrefresh,
	isendwin = CLIB.isendwin,
	noqiflush_sp = CLIB.noqiflush_sp,
	scr_init_sp = CLIB.scr_init_sp,
	__uflow = CLIB.__uflow,
	killchar_sp = CLIB.killchar_sp,
	getmaxx = CLIB.getmaxx,
	addchstr = CLIB.addchstr,
	_IO_flockfile = CLIB._IO_flockfile,
	assume_default_colors_sp = CLIB.assume_default_colors_sp,
	leaveok = CLIB.leaveok,
	insnstr = CLIB.insnstr,
	pclose = CLIB.pclose,
	vsprintf = CLIB.vsprintf,
	bkgd = CLIB.bkgd,
	slk_attron_sp = CLIB.slk_attron_sp,
	freopen = CLIB.freopen,
	idlok = CLIB.idlok,
	resizeterm_sp = CLIB.resizeterm_sp,
	chgat = CLIB.chgat,
	has_key_sp = CLIB.has_key_sp,
	slk_attroff_sp = CLIB.slk_attroff_sp,
	winch = CLIB.winch,
	getchar = CLIB.getchar,
	use_default_colors = CLIB.use_default_colors,
	unctrl = CLIB.unctrl,
	wprintw = CLIB.wprintw,
	beep_sp = CLIB.beep_sp,
	def_shell_mode_sp = CLIB.def_shell_mode_sp,
	getattrs = CLIB.getattrs,
	slk_attr_on = CLIB.slk_attr_on,
	_IO_vfscanf = CLIB._IO_vfscanf,
	fputs = CLIB.fputs,
	wbkgd = CLIB.wbkgd,
	curs_set_sp = CLIB.curs_set_sp,
	wchgat = CLIB.wchgat,
	mousemask_sp = CLIB.mousemask_sp,
	tmpnam_r = CLIB.tmpnam_r,
	fread_unlocked = CLIB.fread_unlocked,
	resize_term_sp = CLIB.resize_term_sp,
	instr = CLIB.instr,
	_IO_vfprintf = CLIB._IO_vfprintf,
	winchstr = CLIB.winchstr,
	idcok = CLIB.idcok,
	vw_scanw = CLIB.vw_scanw,
	getstr = CLIB.getstr,
	remove = CLIB.remove,
	use_env_sp = CLIB.use_env_sp,
	wattr_set = CLIB.wattr_set,
	_IO_putc = CLIB._IO_putc,
	slk_attrset_sp = CLIB.slk_attrset_sp,
	getparx = CLIB.getparx,
	fread = CLIB.fread,
	slk_attron = CLIB.slk_attron,
	tigetstr = CLIB.tigetstr,
	slk_refresh = CLIB.slk_refresh,
	vw_printw = CLIB.vw_printw,
	wattroff = CLIB.wattroff,
	clrtobot = CLIB.clrtobot,
	attron = CLIB.attron,
	touchline = CLIB.touchline,
	_traceattr = CLIB._traceattr,
	__overflow = CLIB.__overflow,
	raw = CLIB.raw,
	scanw = CLIB.scanw,
	nonl = CLIB.nonl,
	wscanw = CLIB.wscanw,
	getwin = CLIB.getwin,
	innstr = CLIB.innstr,
	raw_sp = CLIB.raw_sp,
	attroff = CLIB.attroff,
	scrl = CLIB.scrl,
	resetty_sp = CLIB.resetty_sp,
	use_legacy_coding = CLIB.use_legacy_coding,
	scr_restore = CLIB.scr_restore,
	_tracechtype = CLIB._tracechtype,
	use_window = CLIB.use_window,
	clear = CLIB.clear,
	bkgdset = CLIB.bkgdset,
	mvaddchnstr = CLIB.mvaddchnstr,
	wattrset = CLIB.wattrset,
	vfscanf = CLIB.vfscanf,
	keyok_sp = CLIB.keyok_sp,
	mouseinterval_sp = CLIB.mouseinterval_sp,
	is_keypad = CLIB.is_keypad,
	perror = CLIB.perror,
	define_key = CLIB.define_key,
	is_notimeout = CLIB.is_notimeout,
	trace = CLIB.trace,
	noecho_sp = CLIB.noecho_sp,
	use_screen = CLIB.use_screen,
	ferror = CLIB.ferror,
	syncok = CLIB.syncok,
	is_immedok = CLIB.is_immedok,
	addch = CLIB.addch,
	is_scrollok = CLIB.is_scrollok,
	endwin = CLIB.endwin,
	set_tabsize_sp = CLIB.set_tabsize_sp,
	ungetmouse_sp = CLIB.ungetmouse_sp,
	mvwin = CLIB.mvwin,
	mvaddnstr = CLIB.mvaddnstr,
	popen = CLIB.popen,
	cbreak_sp = CLIB.cbreak_sp,
	mvcur_sp = CLIB.mvcur_sp,
	clearerr = CLIB.clearerr,
	nl = CLIB.nl,
	ferror_unlocked = CLIB.ferror_unlocked,
	slk_restore_sp = CLIB.slk_restore_sp,
	noraw_sp = CLIB.noraw_sp,
	wbkgdset = CLIB.wbkgdset,
	werase = CLIB.werase,
	erasechar_sp = CLIB.erasechar_sp,
	wattr_on = CLIB.wattr_on,
	is_leaveok = CLIB.is_leaveok,
	delscreen = CLIB.delscreen,
	nofilter_sp = CLIB.nofilter_sp,
	delwin = CLIB.delwin,
	wborder = CLIB.wborder,
	renameat = CLIB.renameat,
	mvinchnstr = CLIB.mvinchnstr,
	init_color_sp = CLIB.init_color_sp,
	derwin = CLIB.derwin,
	def_prog_mode = CLIB.def_prog_mode,
	mvinstr = CLIB.mvinstr,
	ctermid = CLIB.ctermid,
	termname_sp = CLIB.termname_sp,
	erase = CLIB.erase,
	has_key = CLIB.has_key,
	scr_set = CLIB.scr_set,
	set_escdelay_sp = CLIB.set_escdelay_sp,
	putc_unlocked = CLIB.putc_unlocked,
	wscrl = CLIB.wscrl,
	mvscanw = CLIB.mvscanw,
	setlinebuf = CLIB.setlinebuf,
	has_colors_sp = CLIB.has_colors_sp,
	refresh = CLIB.refresh,
	mvaddch = CLIB.mvaddch,
	scanf = CLIB.scanf,
	slk_touch = CLIB.slk_touch,
	mouseinterval = CLIB.mouseinterval,
	getline = CLIB.getline,
	mvwchgat = CLIB.mvwchgat,
	wattr_off = CLIB.wattr_off,
	_IO_seekoff = CLIB._IO_seekoff,
	_IO_ftrylockfile = CLIB._IO_ftrylockfile,
	mvinnstr = CLIB.mvinnstr,
	fwrite_unlocked = CLIB.fwrite_unlocked,
	has_colors = CLIB.has_colors,
	_IO_getc = CLIB._IO_getc,
	delay_output_sp = CLIB.delay_output_sp,
	wstandout = CLIB.wstandout,
	echo_sp = CLIB.echo_sp,
	qiflush = CLIB.qiflush,
	ftell = CLIB.ftell,
	mvwinsch = CLIB.mvwinsch,
	set_escdelay = CLIB.set_escdelay,
	slk_label = CLIB.slk_label,
	attr_on = CLIB.attr_on,
	longname = CLIB.longname,
	doupdate_sp = CLIB.doupdate_sp,
	sscanf = CLIB.sscanf,
	_IO_feof = CLIB._IO_feof,
	wresize = CLIB.wresize,
	init_color = CLIB.init_color,
	mvwinchstr = CLIB.mvwinchstr,
	def_prog_mode_sp = CLIB.def_prog_mode_sp,
	deleteln = CLIB.deleteln,
	dprintf = CLIB.dprintf,
	mvinsnstr = CLIB.mvinsnstr,
	winsertln = CLIB.winsertln,
	wtimeout = CLIB.wtimeout,
	is_linetouched = CLIB.is_linetouched,
	mvinchstr = CLIB.mvinchstr,
	tmpnam = CLIB.tmpnam,
	mvhline = CLIB.mvhline,
	keybound_sp = CLIB.keybound_sp,
	getdelim = CLIB.getdelim,
	pair_content = CLIB.pair_content,
	mvwaddchnstr = CLIB.mvwaddchnstr,
	assume_default_colors = CLIB.assume_default_colors,
	typeahead = CLIB.typeahead,
	delch = CLIB.delch,
	reset_prog_mode_sp = CLIB.reset_prog_mode_sp,
	nofilter = CLIB.nofilter,
	wmouse_trafo = CLIB.wmouse_trafo,
	getbegy = CLIB.getbegy,
	fgetc_unlocked = CLIB.fgetc_unlocked,
	mvwaddchstr = CLIB.mvwaddchstr,
	fputc = CLIB.fputc,
	getchar_unlocked = CLIB.getchar_unlocked,
	keyname = CLIB.keyname,
	pechochar = CLIB.pechochar,
	vdprintf = CLIB.vdprintf,
	newterm_sp = CLIB.newterm_sp,
	fdopen = CLIB.fdopen,
	fileno_unlocked = CLIB.fileno_unlocked,
	init_pair = CLIB.init_pair,
	slk_noutrefresh_sp = CLIB.slk_noutrefresh_sp,
	new_prescr = CLIB.new_prescr,
	savetty = CLIB.savetty,
	vidputs_sp = CLIB.vidputs_sp,
	wrefresh = CLIB.wrefresh,
	curs_set = CLIB.curs_set,
	termattrs = CLIB.termattrs,
	box = CLIB.box,
	__io_seek_fn = CLIB.__io_seek_fn,
	setvbuf = CLIB.setvbuf,
	reset_shell_mode_sp = CLIB.reset_shell_mode_sp,
	halfdelay_sp = CLIB.halfdelay_sp,
	wgetdelay = CLIB.wgetdelay,
	killchar = CLIB.killchar,
	putwin = CLIB.putwin,
	fileno = CLIB.fileno,
	waddnstr = CLIB.waddnstr,
	wattron = CLIB.wattron,
	slk_init_sp = CLIB.slk_init_sp,
	wnoutrefresh = CLIB.wnoutrefresh,
	PAIR_NUMBER = CLIB.PAIR_NUMBER,
	winsnstr = CLIB.winsnstr,
	wvline = CLIB.wvline,
	doupdate = CLIB.doupdate,
	addstr = CLIB.addstr,
	has_il = CLIB.has_il,
	def_shell_mode = CLIB.def_shell_mode,
	keypad = CLIB.keypad,
	use_tioctl = CLIB.use_tioctl,
	is_term_resized = CLIB.is_term_resized,
	getw = CLIB.getw,
	slk_init = CLIB.slk_init,
	newwin_sp = CLIB.newwin_sp,
	_IO_ferror = CLIB._IO_ferror,
	vsscanf = CLIB.vsscanf,
	scr_dump = CLIB.scr_dump,
	is_term_resized_sp = CLIB.is_term_resized_sp,
	insertln = CLIB.insertln,
	nocbreak_sp = CLIB.nocbreak_sp,
	scr_restore_sp = CLIB.scr_restore_sp,
	getpary = CLIB.getpary,
	mcprint_sp = CLIB.mcprint_sp,
	use_extended_names = CLIB.use_extended_names,
	mcprint = CLIB.mcprint,
	_IO_sgetn = CLIB._IO_sgetn,
	slk_set = CLIB.slk_set,
	has_mouse_sp = CLIB.has_mouse_sp,
	get_escdelay_sp = CLIB.get_escdelay_sp,
	feof_unlocked = CLIB.feof_unlocked,
	noecho = CLIB.noecho,
	mvchgat = CLIB.mvchgat,
	funlockfile = CLIB.funlockfile,
	newpad = CLIB.newpad,
	fflush_unlocked = CLIB.fflush_unlocked,
	move = CLIB.move,
	printf = CLIB.printf,
	vwprintw = CLIB.vwprintw,
	putchar = CLIB.putchar,
	isendwin_sp = CLIB.isendwin_sp,
	slk_color = CLIB.slk_color,
	wcursyncup = CLIB.wcursyncup,
	fputc_unlocked = CLIB.fputc_unlocked,
	__io_close_fn = CLIB.__io_close_fn,
	initscr = CLIB.initscr,
	mvwinnstr = CLIB.mvwinnstr,
	clearerr_unlocked = CLIB.clearerr_unlocked,
	rename = CLIB.rename,
	insdelln = CLIB.insdelln,
	is_syncok = CLIB.is_syncok,
	winstr = CLIB.winstr,
	color_content = CLIB.color_content,
	baudrate = CLIB.baudrate,
	_IO_free_backup_area = CLIB._IO_free_backup_area,
	clrtoeol = CLIB.clrtoeol,
	hline = CLIB.hline,
	fgetpos = CLIB.fgetpos,
	wsyncup = CLIB.wsyncup,
	insch = CLIB.insch,
	get_escdelay = CLIB.get_escdelay,
	__io_write_fn = CLIB.__io_write_fn,
	mvprintw = CLIB.mvprintw,
	attr_off = CLIB.attr_off,
	echo = CLIB.echo,
	putchar_unlocked = CLIB.putchar_unlocked,
	color_content_sp = CLIB.color_content_sp,
	fseeko = CLIB.fseeko,
	clearok = CLIB.clearok,
	getwin_sp = CLIB.getwin_sp,
	setbuf = CLIB.setbuf,
	winsch = CLIB.winsch,
	vline = CLIB.vline,
	fclose = CLIB.fclose,
	newpad_sp = CLIB.newpad_sp,
	fgets = CLIB.fgets,
	is_idlok = CLIB.is_idlok,
	wdeleteln = CLIB.wdeleteln,
	fscanf = CLIB.fscanf,
	flash_sp = CLIB.flash_sp,
	snprintf = CLIB.snprintf,
	napms = CLIB.napms,
	slk_attrset = CLIB.slk_attrset,
	newwin = CLIB.newwin,
	overwrite = CLIB.overwrite,
	flash = CLIB.flash,
	dupwin = CLIB.dupwin,
	attr_set = CLIB.attr_set,
	notimeout = CLIB.notimeout,
	keyok = CLIB.keyok,
	has_il_sp = CLIB.has_il_sp,
	_IO_padn = CLIB._IO_padn,
	resizeterm = CLIB.resizeterm,
	termattrs_sp = CLIB.termattrs_sp,
	mvaddchstr = CLIB.mvaddchstr,
	is_idcok = CLIB.is_idcok,
	overlay = CLIB.overlay,
	reset_prog_mode = CLIB.reset_prog_mode,
	typeahead_sp = CLIB.typeahead_sp,
	filter = CLIB.filter,
	immedok = CLIB.immedok,
	setscrreg = CLIB.setscrreg,
	mvinsch = CLIB.mvinsch,
	redrawwin = CLIB.redrawwin,
	curses_version = CLIB.curses_version,
	ungetc = CLIB.ungetc,
	fmemopen = CLIB.fmemopen,
	slk_clear = CLIB.slk_clear,
	mvinch = CLIB.mvinch,
	getc = CLIB.getc,
	wgetparent = CLIB.wgetparent,
	setbuffer = CLIB.setbuffer,
	pair_content_sp = CLIB.pair_content_sp,
	puts = CLIB.puts,
	getbegx = CLIB.getbegx,
	beep = CLIB.beep,
	fflush = CLIB.fflush,
	__getdelim = CLIB.__getdelim,
	printw = CLIB.printw,
	is_subwin = CLIB.is_subwin,
	wclear = CLIB.wclear,
	slk_attr = CLIB.slk_attr,
	slk_attr_set = CLIB.slk_attr_set,
	vidattr = CLIB.vidattr,
	erasechar = CLIB.erasechar,
	mvwinch = CLIB.mvwinch,
	scr_set_sp = CLIB.scr_set_sp,
	touchwin = CLIB.touchwin,
	mvvline = CLIB.mvvline,
	is_wintouched = CLIB.is_wintouched,
	getcurx = CLIB.getcurx,
	slk_attroff = CLIB.slk_attroff,
	set_tabsize = CLIB.set_tabsize,
	scroll = CLIB.scroll,
	newterm = CLIB.newterm,
	tigetnum = CLIB.tigetnum,
	slk_set_sp = CLIB.slk_set_sp,
	waddch = CLIB.waddch,
	resize_term = CLIB.resize_term,
	fprintf = CLIB.fprintf,
	untouchwin = CLIB.untouchwin,
	use_env = CLIB.use_env,
	subwin = CLIB.subwin,
	rewind = CLIB.rewind,
	can_change_color = CLIB.can_change_color,
	is_pad = CLIB.is_pad,
	start_color = CLIB.start_color,
	waddchnstr = CLIB.waddchnstr,
	COLOR_PAIR = CLIB.COLOR_PAIR,
	wattr_get = CLIB.wattr_get,
	attr_get = CLIB.attr_get,
	mvwprintw = CLIB.mvwprintw,
	cbreak = CLIB.cbreak,
	wdelch = CLIB.wdelch,
	key_defined_sp = CLIB.key_defined_sp,
	tiparm = CLIB.tiparm,
	winchnstr = CLIB.winchnstr,
	winnstr = CLIB.winnstr,
	has_ic = CLIB.has_ic,
	slk_attr_set_sp = CLIB.slk_attr_set_sp,
	winsdelln = CLIB.winsdelln,
	fwrite = CLIB.fwrite,
	wredrawln = CLIB.wredrawln,
	wsyncdown = CLIB.wsyncdown,
	resetty = CLIB.resetty,
	getcury = CLIB.getcury,
	unctrl_sp = CLIB.unctrl_sp,
	keybound = CLIB.keybound,
	fopen = CLIB.fopen,
	slk_color_sp = CLIB.slk_color_sp,
	key_defined = CLIB.key_defined,
	attrset = CLIB.attrset,
	_IO_peekc_locked = CLIB._IO_peekc_locked,
	mvwvline = CLIB.mvwvline,
	tmpfile = CLIB.tmpfile,
	is_cleared = CLIB.is_cleared,
	putp = CLIB.putp,
	echochar = CLIB.echochar,
	insstr = CLIB.insstr,
	baudrate_sp = CLIB.baudrate_sp,
	can_change_color_sp = CLIB.can_change_color_sp,
	filter_sp = CLIB.filter_sp,
	mvcur = CLIB.mvcur,
	ungetch = CLIB.ungetch,
	winsstr = CLIB.winsstr,
	has_ic_sp = CLIB.has_ic_sp,
	init_pair_sp = CLIB.init_pair_sp,
	wcolor_set = CLIB.wcolor_set,
	__underflow = CLIB.__underflow,
	flockfile = CLIB.flockfile,
	slk_restore = CLIB.slk_restore,
	vprintf = CLIB.vprintf,
	nonl_sp = CLIB.nonl_sp,
	getnstr = CLIB.getnstr,
	mvwaddstr = CLIB.mvwaddstr,
	wgetch = CLIB.wgetch,
	wmove = CLIB.wmove,
	timeout = CLIB.timeout,
	wgetstr = CLIB.wgetstr,
	ungetch_sp = CLIB.ungetch_sp,
	intrflush = CLIB.intrflush,
	start_color_sp = CLIB.start_color_sp,
	vidattr_sp = CLIB.vidattr_sp,
	define_key_sp = CLIB.define_key_sp,
	nocbreak = CLIB.nocbreak,
	ftello = CLIB.ftello,
	has_mouse = CLIB.has_mouse,
}
function library.freeconsole()
	if jit.os == "Windows" then
		ffi.cdef("int FreeConsole();")
		ffi.C.FreeConsole()
	end
end
return library
