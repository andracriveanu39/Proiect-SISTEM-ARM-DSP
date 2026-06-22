/*	
	Acest program integreaza functiile AGC, ALE si MF
	pentru procesorul ADSP2181, selectionabile via PF_DATA.
*/

#include    "def2181.h"    

#define PORT_OUT 0xFF
#define PORT_IN 0x1FF

#define M_AGC_MAX 128 //de inlocuit in vectorii care le folosesc pentru alocare fixa de mem
#define N_ALE_MAX 256 
#define W_MF_MAX 17

.SECTION/DM		buf_var1;
.var    rx_buf[3];      /* Status + L data + R data */

.SECTION/DM		buf_var2;
.var    tx_buf[3] = 0xc000, 0x0000, 0x0000;

.SECTION/DM		buf_var3;
.var    init_cmds[13] = 0xc002, 0xc102, 0xc288, 0xc388, 0xc488, 0xc588, 0xc680, 0xc780, 0xc85c, 0xc909, 0xca00, 0xcc40, 0xcd00;

.SECTION/DM		data1;
/* --- Variabile de Control --- */
.var    stat_flag;
.var    PF_input;       /* Variabila pentru citirea portului */
.var    id_dsp;
.var    cda;
.var    canal;
.var    temp_out;
.var 	temp_in;
.var port_intrare; //voi stoca in data memory valoarea de pe portul de intrare
.var TAB_AFIS[4] = 0x40, 0x79, 0x24, 0x30; //afisez 0, 1, 2 sau 3 in fucntie de filtrul de ruleaza
//
.var cda_prev;

/* --- Variabile AGC --- */
.var M_AGC;
.var K_AGC;
.var    ref_agc;        /* Nivelul de Referinta (R) */
.var    mu_agc;         /* Pasul de adaptare (m) */

.var val_M_AGC[8] = 4, 1, 2, 8, 16, 32, 64, 128; //K va fi log2
.var val_K_AGC[8] = 2, 0, 1, 3, 4, 5, 6, 7;
.var val_mu_agc[8] =  0.05r, 0.0r, 0.001r, 0.015r, 0.01r, 0.1r, 0.5r, 0.9r;
.var val_ref_agc[4]= 0.5r, 0.2r, 0.7r, 0.95r;

.var/circ delay_agc[M_AGC_MAX];
.var    S_agc;          /* Suma / Media modulelor */

//
.var agc_flag_init;
.var    g_int_agc;          /* Castigul curent g(n) int  ax1 */
.var    g_fr_agc;          /* Castigul curent g(n)  fr  - ay1*/
//

/* --- Variabile MF --- */
.var W_MF;
.var K_MF;
.var val_W_MF[8] = 3, 5, 7, 9, 11, 13, 15, 17; // K va fi W>>1

.var/circ delay_mf[W_MF_MAX];
.var    delay_sorted[W_MF_MAX];
.var    mf_flag_init;

/* --- Variabile ALE --- */
.var N_ALE;
.var D_ALE;
.var mu_ale;
.var lambda_ale;

.var val_N_ALE[4] = 256, 20, 30, 128; 
.var val_D_ALE[4] = 128, 5, 50, 256;
.var val_mu_ale[4] = 0.01r, 0.001r, 0.05r, 0.1r; //pastrez pe toate SW 0 valorile default
.var val_lambda_ale[4] = 0.01r, 0.001r, 0.1r, 0.5r;

.var/circ fir_d[N_ALE_MAX];
.var/circ input_ale[N_ALE_MAX];

.var ale_flag_init;

.SECTION/PM		pm_da;
/* --- Variabile PM ALE --- */
.var/circ hh[N_ALE_MAX];


/*** Interrupt Vector Table ***/
.SECTION/PM     interrupts;
        jump start;         rti; rti; rti;  /*00: reset */
        //rti;                rti; rti; rti;  /*04: IRQ2 */
        jump input_samples; rti; rti; rti;  /*08: IRQL1 (SPORT0 rx) */
        rti;                rti; rti; rti;  
        rti;                rti; rti; rti;  /*0c: IRQL0 */
        ar = dm(stat_flag);                 /*10: SPORT0 tx */
        ar = pass ar;
        if eq rti;
        jump next_cmd;
        jump input_samples;                 /*14: SPORT0 rx */
        rti; rti; rti;
        rti;                rti; rti; rti;  /*18: IRQE */
        rti;                rti; rti; rti;  /*1c: BDMA */
        rti;                rti; rti; rti;  /*20: SPORT1 tx or IRQ1 */
        rti;                rti; rti; rti;  /*24: SPORT1 rx or IRQ0 */
        nop;                rti; rti; rti;  /*28: timer */
        rti;                rti; rti; rti;  /*2c: power down */

.SECTION/PM		seg_code;

/*******************************************************************************
 * INITIALIZARI SISTEM SI FILTRE
 *******************************************************************************/
start:
        ax0 = b#0000100000000000;
        dm (Sys_Ctrl_Reg) = ax0;
        ena timer;

        i5 = rx_buf;  l5 = LENGTH(rx_buf);
        i6 = tx_buf;  l6 = LENGTH(tx_buf);
        i3 = init_cmds; l3 = LENGTH(init_cmds);

        m1 = 1; m5 = 1;

        /* SPORT0 Config */
        ax0 = b#0000110011010111;   dm (Sport0_Autobuf_Ctrl) = ax0;
        ax0 = 0;                    dm (Sport0_Rfsdiv) = ax0;
        ax0 = 0;                    dm (Sport0_Sclkdiv) = ax0;
        ax0 = b#1000011000001111;   dm (Sport0_Ctrl_Reg) = ax0;
        ax0 = b#0000000000000111;   dm (Sport0_Tx_Words0) = ax0;
        ax0 = b#0000000000000111;   dm (Sport0_Tx_Words1) = ax0;
        ax0 = b#0000000000000111;   dm (Sport0_Rx_Words0) = ax0;
        ax0 = b#0000000000000111;   dm (Sport0_Rx_Words1) = ax0;

        /* System Config */
        ax0 = b#0001100000000000;   dm (Sys_Ctrl_Reg) = ax0;
        ifc = b#00000011111110;     
        nop;
        icntl = b#00010;
        mstat = b#1100000;
        
        jump skip;

/*******************************************************************************
 * INITIALIZARE CODEC AD1847
 *******************************************************************************/
        ax0 = 1; dm(stat_flag) = ax0; ena ints; imask = b#0001000001;
        ax0 = dm (i6, m5); tx0 = ax0;
check_init: 
        ax0 = dm (stat_flag); af = pass ax0; if ne jump check_init;
        ay0 = 2;
check_aci1: 
        ax0 = dm (rx_buf); ar = ax0 and ay0; if eq jump check_aci1;
check_aci2: 
        ax0 = dm (rx_buf); ar = ax0 and ay0; if ne jump check_aci2; idle;
        ay0 = 0xbf3f; ax0 = dm (init_cmds + 6); ar = ax0 AND ay0; dm (tx_buf) = ar; idle;
        ax0 = dm (init_cmds + 7); ar = ax0 AND ay0; dm (tx_buf) = ar; idle;
        ifc = b#00000011111110; nop; imask = b#0001100001;

skip: 
        imask = 0x200;
        si=0xFFFF; dm(Dm_Wait_Reg)=si;
        /* Setare PF ports exclusiv ca INTRARI */
        si=0x0000; dm(Prog_Flag_Comp_Sel_Ctrl)=si;
        
//
si=3;
dm(cda_prev)=si;

wt:     
        nop;
        jump wt;

/*------------------------------------------------------------------------------
 - ISR SPORT0: Citire si rutare comanda
 ------------------------------------------------------------------------------*/
input_samples:
        ena sec_reg;

        /* ========================================================= */
        /* PF inputs 0-7                                             */
        /* ========================================================= */
        ax0=dm(Prog_Flag_Data);
        ay0=0x00FF;            
        ar=ax0 and ay0;
        dm(PF_input)=ar;       

        /* ========================================================= */
        /* Parsare Comanda din PF_input                              */
        /* ========================================================= */
        ax0 = dm(PF_input);

        /* Validare (D7, D6 == 11) */
        ay0 = 0x00C0; ar = ax0 AND ay0; ay1 = 0x00C0; ar = ar - ay1;
        if ne jump invalid_cmd;

        /* ID DSP (D2, D1, D0 == 001) */
        ay0 = 0x0007; ar = ax0 AND ay0; dm(id_dsp) = ar; ay1 = 0x0001; ar = ar - ay1;
        if ne jump invalid_cmd;

        /* Canal (D3) */
        ay0 = 0x0008; ar = ax0 AND ay0; se = -3; sr = lshift ar (lo); dm(canal) = sr0;

        /* Functie CDA (D5, D4) */
        ay0 = 0x0030; ar = ax0 AND ay0; se = -4; sr = lshift ar (lo); dm(cda) = sr0;

        /* PAS 2: Citire Semnal Intrare x(n) -> AR */
        call read_buff;             
		dm(temp_in)=ar;
        /* PAS 3: Decizie Algoritm */
        //
        ax0=dm(cda_prev);
        ay0=dm(cda);
        ar=ax0-ay0;
        if eq jump cont;
        
        si=1;
        dm(agc_flag_init)=si;
        dm(mf_flag_init)=si;
        dm(ale_flag_init)=si;
        si=dm(cda);
        dm(cda_prev)=si;
        
        call afisare;

        cont:
        //////////////CITIRE PORT INTRARE///////////////

        ax0=IO(PORT_IN);
        ay0=0x00FF;            
        ar=ax0 and ay0;
        dm(port_intrare)=ar; 
        //

        ax0 = dm(cda);
        ay0 = 0; ar = ax0 - ay0; if eq jump run_agc;         // 00: AGC
        ay0 = 1; ar = ax0 - ay0; if eq jump run_ale;         // 01: ALE
        ay0 = 2; ar = ax0 - ay0; if eq jump run_mf;          // 10: MF

invalid_cmd:
        call read_buff;        
        si=3;
        call afisare;     
        jump write_out;             // Bypass (out=in)

/* ========================================================================== */
/* AGC ALGORITHM                                                              */
/* ========================================================================== */
run_agc:
        /*
        // --- INITIALIZARE POINTERI ---
        l3 = M_AGC; 
        i3 = delay_agc; 
        m3 = 1;
		*/
		ar=dm(agc_flag_init);
		ar=ar-1;
		if eq call agc_init;
		
       //
/////////////////// determinare parametri 
ax0=dm(port_intrare);

//miu
ay0=0x0038;
ar=ax0 and ay0;
se = -3;
sr = lshift ar (lo);

i0 = val_mu_agc;
m0 = sr0;
modify(i0, m0);

ax1 = dm(i0, m0);
dm(mu_agc) = ax1;

//ref
ay0=0x00C0;
ar=ax0 and ay0;
se = -6;
sr = lshift ar (lo);

i0=val_ref_agc;
m0=sr0;
modify(i0, m0);

ax1=dm(i0, m0);
dm(ref_agc) = ax1;

ax0=dm(K_AGC); //aduc valoarea din memorie intr-un registru ALU
ar=-ax0; //facem negarea in ALU
se=ar; //salvez rezultatul in se 
//il scriu aici si nu in initializari pt ca nu vreau sa l pierd prin shiftarile de mai sus

///

ax1=dm(g_int_agc);
ay1=dm(g_fr_agc);       
       
//mx0=ar; 			// save input
mx0=dm(temp_in);
mr=0;
my0=-1.0r;

ar = ax1-1;
if lt jump g_fr;
mr=mr-mx0*my0 (ss);  // mr1 = x(n);
g_fr:
my1=ay1;	
mr=mr+mx0*my1 (rnd); // mr1= y(n) = interger_part_g(n-1)*x(n-1)+fractional_part_g(n-1)*x(n) <1

//dm(tx_buf+1)=mr1;	// write output
ar=mr1;
call write_out;

//
ar = abs mr1;
sr=ashift ar (hi); // sr1 - scaled input 
dm(i3,m3)=sr1; 	   // update delay line

// compute average
ar=dm(M_AGC);
ar=ar-1;
cntr=ar; // era cntr=M_AGC-1

mr=0, mx0=dm(i3,m3);
do sum until ce;
sum: mr=mr-mx0*my0(ss), mx0=dm(i3,m3);
mr=mr-mx0*my0(rnd);
if mv sat mr;

dm(S_agc)=mr1;

ax0=dm(ref_agc);
ay0=dm(S_agc);
ar=ax0-ay0; // ar=ref-S

// ay1= fractional part of g(n-1);
// ax1 = integer part of g(n-1)

my1=ar;
mx1=dm(mu_agc);

mr=mx1*my1 (ss);	// mr1 = mu*(ref-S)
ar=mr1+ay1;			// ar = g(n-1)+ mu*(ref-S) = g(n)
af = pass ar;		// ar < 0 -> overflow g(n)>1, update integer and fractional parts
if ge jump cont_agc;
// gain correction: ax1=1 -> ax1=1, ay1=0; ax1=1 -> ax1=0; ay1=0x7FFF
af = ax1-1;
if lt jump cor1;
// ax1=1 -> ay1=1.0, ax1=0;
ax1=0;
ay1=0x7fff;
//rti;
jump save_gain;
// ax1=0 -> ay1=0.0, ax1=1
cor1:
ax1=1;
ay1=0;
//rti;
jump save_gain;

cont_agc:
ay1=ar;

save_gain:
dm(g_fr_agc)=ay1;
dm(g_int_agc)=ax1;
rti;

       //

/* ========================================================================== */
/* ALE ALGORITHM */
/* ========================================================================== */
run_ale:
        ena ar_sat;

        ax0=dm(port_intrare);

//lambda
ay0=0x0003;
ar=ax0 and ay0;

i0=val_lambda_ale;
m0=ar;
modify(i0, m0);

ax1=dm(i0, m0);
dm(lambda_ale)=ax1;

//miu
ay0=0x000C;
ar=ax0 and ay0;
se=-2;
sr = lshift ar (lo);

i0=val_mu_ale;
m0=sr0;
modify(i0, m0);

ax1=dm(i0, m0);
dm(mu_ale)=ax1;

        ar=dm(ale_flag_init);
	ar=ar-1;
	if eq call ale_init;

        ar=dm(temp_in);
        dm(i2,m1)=ar;		// actualizeaza linia de intirziere de intrare
	ar=dm(i4,m5);	
	dm(i3,m1)=ar;		// actualizeaza linia de intirziere a FIR
        
	//calculeaza iesirea FIR

	ar=dm(N_ALE);
        ar=ar-1;
        cntr=ar;   //cntr=N_ALE-1
	call fir;		// mr1 - iesirea filtrului

	//scrie iesirea - semnalul "curatat"

        ar=mr1;
	call write_out;	//in mr1 - valoarea dorita pentru scriere

	//calcul eroare

	ar=dm(temp_in);	// in ar - esantionul de intrare
	ay0=ar;
 	ar=ay0-mr1;		// in ar - eroarea
	mx0=ar;
	//ar=dm(N_ALE);
        cntr=dm(N_ALE); 
	my1=dm(mu_ale);
	m4=-1;
	m6=2;
	m3=-1;
	//call lms;		// adapteaza coeficientii FIR
	m7=0;
	mx1=dm(lambda_ale);
	call llms;
        dis sec_reg;           // revine la setul primar de registre
        rti;

// ----------------FIR Subroutine--------------------------
/*
i3 = ^ filter input;
i6 = ^ filter coeficients
m1=1;m5=1;
mr1 = filter output
*/

fir:    mr=0, mx0=dm(i3,m1), my0=pm(i6,m5);
        do sop until ce;
sop:    mr=mr+mx0*my0(ss), mx0=dm(i3,m1), my0=pm(i6,m5);
        mr=mr+mx0*my0(rnd);
        if mv sat mr;
        rts; 

//----------------LLMS Modify Subroutine-----------------------

// h(k) = h(k)*(1-mu*lambda) + mu*x(n-k)*e(n)
/*

mx0 = error
my1 = mu
mx1 = lambda

m7=0;

i3 = ^ filter input;
i6 = ^ filter coeficients
m1=1;m5=1;m4=-1;m6=2;m3=-1;
mr1 = filter output
*/

llms:           mr=mx0*my1(rnd), mx0=dm(i3,m1);   // mr1 = error * mu , mx0= x(n-k)
				my0=mr1;						  // my0 = error*mu
                mr=mx0*my0(rnd), ay0=pm(i6,m5);	  // mr1 = x(n-k)*mu*error, ay0= h(k)
                
                do adaptive1 until ce;
                ar=mr1+ay0, mx0=dm(i3,m1), ay0=pm(i6,m4); //ar = h(k) +x(n-k)*mu*error
               
                mf=mx1*my1(rnd), sr1=pm(i6,m7);	  // mf= mu*lambda, sr1=h(k)
				mr=sr1*mf(rnd); 				  // mr1=h(k)*mu*lambda
				ay1 = mr1;		  				  // ay1=h(k)*mu*lambda
				
                ar=ar-ay1;						  //ar = h(k)-h(k)*mu*lambda+x(n-k)*mu*error
                

adaptive1:      pm(i6,m6)=ar, mr=mx0*my0(rnd);
               
		modify(i3,m3);
                modify(i6,m4);
                rts;

/* ========================================================================== */
/* MEDIAN FILTER ALGORITHM */
/* ========================================================================== */
run_mf:
        ar=dm(mf_flag_init);
	ar=ar-1;
	if eq call mf_init;

ar=dm(temp_in);
dm(i3,m3)=ar; 	 // update delay line

// copy delay to delay sorted
i0=delay_sorted;
//ar=dm(W_MF);
cntr=dm(W_MF);
do copy until ce;
si=dm(i3,m3);
copy:
dm(i0,m3)=si;

i2=delay_sorted;
call sort_sel;
// median value
i0=delay_sorted;
m0=dm(K_MF);
modify(i0,m0);
ar=dm(i0,m3);
call write_out;


rti;


/////////////////////////////////////////////////////////////////////
// sortare prin selectie
sort_sel:
/*
	for (i=0; i<W_MF; i++)
		{
			min=i; 
			for (j=i+1; j<W_MF; j++)
				{
				if A[j]<A[min] min=j;
				}
			tmp=A[i];
			A[i]=A[min];
			A[min]=tmp;
		}
*/


// sorting


// m0 - min
// m1 - (i)
// m2 - (i+1)

// i2 pointer to A

m3=1;

l0=0;
l1=0;

//ar=dm(W_MF);
cntr=dm(W_MF);
// 	for (i=0; i<W_MF-1; i++)
do loop_i until ce;
ax0=dm(W_MF);
ay0=cntr;
ar=ax0-ay0; //i
m0=ar; 		// m0 = min
m1=ar;		// m1 = i

ar=ar+1;
m2=ar;		// m2 = i+1
ay0=m1;
ar=ax0-ay0; 
ar=ar-1;	// # of iteration in loop_j


if eq jump loop_i;

cntr=ar;



i0=i2;
modify(i0,m2);	// i0=A[i+1]

// for (j=i+1; j<W_MF; j++)
	do loop_j until ce;
	//if A[j]<A[min] min=j;
	ax0=dm(i0,m3); // ax0= A[j]
	i1=i2;
	modify(i1,m0);
	ay0=dm(i1,m3); // ay0=A[min]
	ar=ax0-ay0;
	if gt jump loop_j;
	ax0=dm(W_MF);
	ay0=cntr;
	ar=ax0-ay0; //j
	m0=ar; 		// min = j
	loop_j: nop;
/*
tmp=A[i];
A[i]=A[min];
A[min]=tmp;
*/
i1=i2;
modify(i1,m1);
ax1=dm(i1,m3);	// ax1 = tmp = A[i];
i1=i2;
modify(i1,m0);	
ay1=dm(i1,m3);	// ay1=A[min]
i1=i2;
modify(i1,m1);
dm(i1,m3)=ay1;	// A[i]=A[min]
i1=i2;
modify(i1,m0);	
dm(i1,m3)=ax1;	// A[min]=tmp


loop_i: nop;



rts;

/* ========================================================================== */
/* PAS 4: SCRIEREA REZULTATELOR */
/* ========================================================================== */
write_out:
        dm(temp_out) = ar;  

        ax0 = dm(canal);
        ay0 = 0; ar = ax0 - ay0;
        if eq jump write_left;      
        
write_right:
        ar = dm(temp_out);
        dm(tx_buf + 2) = ar;
        //jump end_isr;
        rts;

write_left:
        ar = dm(temp_out);
        dm(tx_buf + 1) = ar;
        rts;
        
//end_isr:
//        dis sec_reg;
//        rti;

/* ========================================================================== */
/* RUTINE AUXILIARE */
/* ========================================================================== */

/* Selectie Citire Canal */
read_buff:
        ax0 = dm(canal); ay0 = 0; ar = ax0 - ay0;
        if eq jump c0;              
        ay0 = 1; ar = ax0 - ay0;
        if eq jump c1;              
c0:     
        ar = dm(rx_buf + 1); rts;
c1:     
        ar = dm(rx_buf + 2); rts;

/* Afisor */
afisare:
	i0=TAB_AFIS;
	m0=si;
	modify(i0, m0);
	si=dm(i0, m0);
	IO(PORT_OUT)=si;
	rts;

/* Rutina Initiere Codec */
next_cmd:
        ena sec_reg; ax0 = dm (i3, m1); dm (tx_buf) = ax0;
        ax0 = i3; ay0 = init_cmds; ar = ax0 - ay0; if gt rti;
        ax0 = 0xaf00; dm (tx_buf) = ax0; ax0 = 0; dm (stat_flag) = ax0; rti;
        
        
        
/////////////////////////////////////////////////////////////

agc_init:

ax0=dm(port_intrare);

//m si k
ay0=0x0007;
ar=ax0 and ay0;

i0=val_M_AGC;
m0=ar;
modify(i0, m0); //trece pe pozitia pe care o vreau 
ax1=dm(i0, m0);
dm(M_AGC)=ax1;

i0=val_K_AGC;
modify(i0, m0);
ax1=dm(i0, m0);
dm(K_AGC)=ax1;

//restul initializarilor
si=0;
dm(S_agc)=si;
dm(g_fr_agc)=si; // g(n-1) fractional part 
dm(g_int_agc)=si; // g(n-1) integer part 
/*si=0.5r; nu mai hardcodam
dm(ref_agc)=si;
si=0.05r;
dm(mu_agc)=si;
*/

l3=dm(M_AGC);
//l3=ax0;
i3=delay_agc;
m3=1;

si=0;
dm(agc_flag_init)=si;

rts;


mf_init:

ax0=dm(port_intrare);

//W
ay0=0x0007;
ar=ax0 and ay0;

i4=val_W_MF;
m4=ar;
modify(i4, m4);

ax1=dm(i4, m4);
dm(W_MF)=ax1;

//K
se=-1;
ar=ax1;
sr=lshift ar (lo);

dm(K_MF)=sr0;

l3=dm(W_MF);
//l3=ax0; 
i3=delay_mf;
m3=1;
l0=0;

si=0;
dm(mf_flag_init)=si;

rts;

ale_init: 

ax0=dm(port_intrare);

//D_ALE
ay0=0x0030;
ar=ax0 and ay0;
se=-4;
sr = lshift ar (lo);

i0=val_D_ALE;
m0=sr0;
modify(i0, m0);

ax1=dm(i0, m0);
dm(D_ALE)=ax1;

//N_ALE
ay0=0x00C0;
ar=ax0 and ay0;
se=-6;
sr = lshift ar (lo);

i0=val_N_ALE;
m0=sr0;
modify(i0, m0);

ax1=dm(i0, m0);
dm(N_ALE)=ax1;

	//initializari registre
        /*si=0.01r;
        dm(mu_ale) = si;
        dm(lambda_ale) = si;
        */
	m1=1;
	m5=1;
	si=0;
	i2=input_ale;
    l2=dm(N_ALE);
	//l2=ax0;

    //ay0=dm(D_ALE);
    m4=dm(D_ALE);
    i4=input_ale;
    modify(i4, m4); //am inlocuit i4=input_ale+D_ALE acum ca D_ALE este .var
	
	l4=dm(N_ALE);
	i3=fir_d;
	l3=dm(N_ALE);
	i6=hh;
	l6=dm(N_ALE);

	//initializare linii intirziere si coeficienti

	cntr=dm(N_ALE);
	do init_buf until ce;
	dm(i2,m1)=si;
	dm(i3,m1)=si;
        init_buf:
	pm(i6,m5)=si;

        si=0;
        dm(ale_flag_init)=si;
        rts;
