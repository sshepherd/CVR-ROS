/*
 * Copyright (c) 2003, 2007-8 Matteo Frigo
 * Copyright (c) 2003, 2007-8 Massachusetts Institute of Technology
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

/* This file was automatically generated --- DO NOT EDIT */
/* Generated on Sun Jul 12 06:45:12 EDT 2009 */

#include "codelet-rdft.h"

#ifdef HAVE_FMA

/* Generated by: ../../../genfft/gen_hc2c -fma -reorder-insns -schedule-for-pipeline -compact -variables 4 -pipeline-latency 4 -twiddle-log3 -precompute-twiddles -n 20 -dit -name hc2cf2_20 -include hc2cf.h */

/*
 * This function contains 276 FP additions, 198 FP multiplications,
 * (or, 136 additions, 58 multiplications, 140 fused multiply/add),
 * 142 stack variables, 4 constants, and 80 memory accesses
 */
#include "hc2cf.h"

static void hc2cf2_20(R *Rp, R *Ip, R *Rm, R *Im, const R *W, stride rs, INT mb, INT me, INT ms)
{
     DK(KP951056516, +0.951056516295153572116439333379382143405698634);
     DK(KP559016994, +0.559016994374947424102293417182819058860154590);
     DK(KP250000000, +0.250000000000000000000000000000000000000000000);
     DK(KP618033988, +0.618033988749894848204586834365638117720309180);
     INT m;
     for (m = mb, W = W + ((mb - 1) * 8); m < me; m = m + 1, Rp = Rp + ms, Ip = Ip + ms, Rm = Rm - ms, Im = Im - ms, W = W + 8, MAKE_VOLATILE_STRIDE(rs)) {
	  E T59, T5i, T5k, T5e, T5c, T5d, T5j, T5f;
	  {
	       E T2, Th, Tf, T6, T5, Tl, T1p, T1n, Ti, T3, Tt, Tv, T24, T1f, T1D;
	       E Tb, T1P, Tm, T21, T1b, T7, T1A, Tw, T1H, T13, TA, T1L, T17, T1S, Tq;
	       E T1o, T2g, T1t, T2c, TO, TK;
	       {
		    E T1e, Ta, Tk, Tg;
		    T2 = W[0];
		    Th = W[3];
		    Tf = W[2];
		    T6 = W[5];
		    T5 = W[1];
		    Tk = T2 * Th;
		    Tg = T2 * Tf;
		    T1e = Tf * T6;
		    Ta = T2 * T6;
		    Tl = FMA(T5, Tf, Tk);
		    T1p = FNMS(T5, Tf, Tk);
		    T1n = FMA(T5, Th, Tg);
		    Ti = FNMS(T5, Th, Tg);
		    T3 = W[4];
		    Tt = W[6];
		    Tv = W[7];
		    {
			 E Tp, Tj, TN, TJ;
			 Tp = Ti * T6;
			 T24 = FMA(Th, T3, T1e);
			 T1f = FNMS(Th, T3, T1e);
			 T1D = FNMS(T5, T3, Ta);
			 Tb = FMA(T5, T3, Ta);
			 Tj = Ti * T3;
			 {
			      E T1a, T4, Tu, T1G;
			      T1a = Tf * T3;
			      T4 = T2 * T3;
			      Tu = Ti * Tt;
			      T1G = T2 * Tt;
			      {
				   E T12, Tz, T1K, T16;
				   T12 = Tf * Tt;
				   Tz = Ti * Tv;
				   T1K = T2 * Tv;
				   T16 = Tf * Tv;
				   T1P = FNMS(Tl, T6, Tj);
				   Tm = FMA(Tl, T6, Tj);
				   T21 = FNMS(Th, T6, T1a);
				   T1b = FMA(Th, T6, T1a);
				   T7 = FNMS(T5, T6, T4);
				   T1A = FMA(T5, T6, T4);
				   Tw = FMA(Tl, Tv, Tu);
				   T1H = FMA(T5, Tv, T1G);
				   T13 = FMA(Th, Tv, T12);
				   TA = FNMS(Tl, Tt, Tz);
				   T1L = FNMS(T5, Tt, T1K);
				   T17 = FNMS(Th, Tt, T16);
				   T1S = FMA(Tl, T3, Tp);
				   Tq = FNMS(Tl, T3, Tp);
			      }
			 }
			 T1o = T1n * T3;
			 T2g = T1n * Tv;
			 TN = Tm * Tv;
			 TJ = Tm * Tt;
			 T1t = T1n * T6;
			 T2c = T1n * Tt;
			 TO = FNMS(Tq, Tt, TN);
			 TK = FMA(Tq, Tv, TJ);
		    }
	       }
	       {
		    E Te, T2C, T4L, T57, T58, TD, T2H, T4H, T3J, T3Z, T11, T2v, T2P, T3P, T4d;
		    E T4z, T3n, T43, T2r, T2z, T3b, T3T, T4n, T4v, T3u, T42, T20, T2y, T34, T3S;
		    E T4k, T4w, T1c, T19, T1d, T3y, T1w, T2U, T1g, T1j, T1l;
		    {
			 E T2d, T2h, T2k, T1q, T1u, T2n, TL, TI, TM, T3F, TZ, T2N, TP, TS, TU;
			 {
			      E T1, T4K, T8, T9, Tc;
			      T1 = Rp[0];
			      T4K = Rm[0];
			      T8 = Rp[WS(rs, 5)];
			      T2d = FMA(T1p, Tv, T2c);
			      T2h = FNMS(T1p, Tt, T2g);
			      T2k = FMA(T1p, T6, T1o);
			      T1q = FNMS(T1p, T6, T1o);
			      T1u = FMA(T1p, T3, T1t);
			      T2n = FNMS(T1p, T3, T1t);
			      T9 = T7 * T8;
			      Tc = Rm[WS(rs, 5)];
			      {
				   E Tx, Ts, T2F, TC, T2E;
				   {
					E Tn, Tr, To, T2D, T4J, Ty, TB, Td, T4I;
					Tn = Ip[WS(rs, 2)];
					Tr = Im[WS(rs, 2)];
					Tx = Ip[WS(rs, 7)];
					Td = FMA(Tb, Tc, T9);
					T4I = T7 * Tc;
					To = Tm * Tn;
					T2D = Tm * Tr;
					Te = T1 + Td;
					T2C = T1 - Td;
					T4J = FNMS(Tb, T8, T4I);
					Ty = Tw * Tx;
					TB = Im[WS(rs, 7)];
					Ts = FMA(Tq, Tr, To);
					T4L = T4J + T4K;
					T57 = T4K - T4J;
					T2F = Tw * TB;
					TC = FMA(TA, TB, Ty);
					T2E = FNMS(Tq, Tn, T2D);
				   }
				   {
					E TF, TG, TH, TW, TY, T2G, T3E, TX, T2M;
					TF = Rp[WS(rs, 2)];
					T2G = FNMS(TA, Tx, T2F);
					T58 = Ts - TC;
					TD = Ts + TC;
					TG = Ti * TF;
					T2H = T2E - T2G;
					T4H = T2E + T2G;
					TH = Rm[WS(rs, 2)];
					TW = Ip[WS(rs, 9)];
					TY = Im[WS(rs, 9)];
					TL = Rp[WS(rs, 7)];
					TI = FMA(Tl, TH, TG);
					T3E = Ti * TH;
					TX = Tt * TW;
					T2M = Tt * TY;
					TM = TK * TL;
					T3F = FNMS(Tl, TF, T3E);
					TZ = FMA(Tv, TY, TX);
					T2N = FNMS(Tv, TW, T2M);
					TP = Rm[WS(rs, 7)];
					TS = Ip[WS(rs, 4)];
					TU = Im[WS(rs, 4)];
				   }
			      }
			 }
			 {
			      E T27, T26, T28, T3j, T2p, T39, T29, T2e, T2i;
			      {
				   E T22, T23, T25, T2l, T2o, T3i, T2m, T38;
				   {
					E TR, T2J, T3H, TV, T2L, T4b, T3I;
					T22 = Rp[WS(rs, 6)];
					{
					     E TQ, T3G, TT, T2K;
					     TQ = FMA(TO, TP, TM);
					     T3G = TK * TP;
					     TT = T3 * TS;
					     T2K = T3 * TU;
					     TR = TI + TQ;
					     T2J = TI - TQ;
					     T3H = FNMS(TO, TL, T3G);
					     TV = FMA(T6, TU, TT);
					     T2L = FNMS(T6, TS, T2K);
					     T23 = T21 * T22;
					}
					T4b = T3F + T3H;
					T3I = T3F - T3H;
					{
					     E T10, T3D, T4c, T2O;
					     T10 = TV + TZ;
					     T3D = TZ - TV;
					     T4c = T2L + T2N;
					     T2O = T2L - T2N;
					     T3J = T3D - T3I;
					     T3Z = T3I + T3D;
					     T11 = TR - T10;
					     T2v = TR + T10;
					     T2P = T2J - T2O;
					     T3P = T2J + T2O;
					     T4d = T4b + T4c;
					     T4z = T4c - T4b;
					     T25 = Rm[WS(rs, 6)];
					}
				   }
				   T2l = Ip[WS(rs, 3)];
				   T2o = Im[WS(rs, 3)];
				   T27 = Rp[WS(rs, 1)];
				   T26 = FMA(T24, T25, T23);
				   T3i = T21 * T25;
				   T2m = T2k * T2l;
				   T38 = T2k * T2o;
				   T28 = T1n * T27;
				   T3j = FNMS(T24, T22, T3i);
				   T2p = FMA(T2n, T2o, T2m);
				   T39 = FNMS(T2n, T2l, T38);
				   T29 = Rm[WS(rs, 1)];
				   T2e = Ip[WS(rs, 8)];
				   T2i = Im[WS(rs, 8)];
			      }
			      {
				   E T1I, T1F, T1J, T3q, T1Y, T32, T1M, T1Q, T1T;
				   {
					E T1B, T1C, T1E, T1V, T1X, T3p, T1W, T31;
					{
					     E T2b, T35, T3l, T2j, T37, T4l, T3m;
					     T1B = Rp[WS(rs, 4)];
					     {
						  E T2a, T3k, T2f, T36;
						  T2a = FMA(T1p, T29, T28);
						  T3k = T1n * T29;
						  T2f = T2d * T2e;
						  T36 = T2d * T2i;
						  T2b = T26 + T2a;
						  T35 = T26 - T2a;
						  T3l = FNMS(T1p, T27, T3k);
						  T2j = FMA(T2h, T2i, T2f);
						  T37 = FNMS(T2h, T2e, T36);
						  T1C = T1A * T1B;
					     }
					     T4l = T3j + T3l;
					     T3m = T3j - T3l;
					     {
						  E T2q, T3h, T4m, T3a;
						  T2q = T2j + T2p;
						  T3h = T2p - T2j;
						  T4m = T37 + T39;
						  T3a = T37 - T39;
						  T3n = T3h - T3m;
						  T43 = T3m + T3h;
						  T2r = T2b - T2q;
						  T2z = T2b + T2q;
						  T3b = T35 - T3a;
						  T3T = T35 + T3a;
						  T4n = T4l + T4m;
						  T4v = T4m - T4l;
						  T1E = Rm[WS(rs, 4)];
					     }
					}
					T1V = Ip[WS(rs, 1)];
					T1X = Im[WS(rs, 1)];
					T1I = Rp[WS(rs, 9)];
					T1F = FMA(T1D, T1E, T1C);
					T3p = T1A * T1E;
					T1W = Tf * T1V;
					T31 = Tf * T1X;
					T1J = T1H * T1I;
					T3q = FNMS(T1D, T1B, T3p);
					T1Y = FMA(Th, T1X, T1W);
					T32 = FNMS(Th, T1V, T31);
					T1M = Rm[WS(rs, 9)];
					T1Q = Ip[WS(rs, 6)];
					T1T = Im[WS(rs, 6)];
				   }
				   {
					E T14, T15, T18, T1r, T1v, T3x, T1s, T2T;
					{
					     E T1O, T2Y, T3s, T1U, T30, T4i, T3t;
					     T14 = Rp[WS(rs, 8)];
					     {
						  E T1N, T3r, T1R, T2Z;
						  T1N = FMA(T1L, T1M, T1J);
						  T3r = T1H * T1M;
						  T1R = T1P * T1Q;
						  T2Z = T1P * T1T;
						  T1O = T1F + T1N;
						  T2Y = T1F - T1N;
						  T3s = FNMS(T1L, T1I, T3r);
						  T1U = FMA(T1S, T1T, T1R);
						  T30 = FNMS(T1S, T1Q, T2Z);
						  T15 = T13 * T14;
					     }
					     T4i = T3q + T3s;
					     T3t = T3q - T3s;
					     {
						  E T1Z, T3o, T4j, T33;
						  T1Z = T1U + T1Y;
						  T3o = T1Y - T1U;
						  T4j = T30 + T32;
						  T33 = T30 - T32;
						  T3u = T3o - T3t;
						  T42 = T3t + T3o;
						  T20 = T1O - T1Z;
						  T2y = T1O + T1Z;
						  T34 = T2Y - T33;
						  T3S = T2Y + T33;
						  T4k = T4i + T4j;
						  T4w = T4j - T4i;
						  T18 = Rm[WS(rs, 8)];
					     }
					}
					T1r = Ip[WS(rs, 5)];
					T1v = Im[WS(rs, 5)];
					T1c = Rp[WS(rs, 3)];
					T19 = FMA(T17, T18, T15);
					T3x = T13 * T18;
					T1s = T1q * T1r;
					T2T = T1q * T1v;
					T1d = T1b * T1c;
					T3y = FNMS(T17, T14, T3x);
					T1w = FMA(T1u, T1v, T1s);
					T2U = FNMS(T1u, T1r, T2T);
					T1g = Rm[WS(rs, 3)];
					T1j = Ip[0];
					T1l = Im[0];
				   }
			      }
			 }
		    }
		    {
			 E T3C, T40, T2W, T3Q, T4M, T4E, T4F, T4U, T4S;
			 {
			      E T4W, T2u, T2w, T4g, T4V, T4D, T4B, T54, T56, T4Y, T4u, T4C;
			      {
				   E T4x, TE, T53, T1z, T2s, T52, T4A, T4t, T4s, T2t;
				   {
					E T1i, T2Q, T3A, T1m, T2S;
					T4x = T4v - T4w;
					T4W = T4w + T4v;
					{
					     E T1h, T3z, T1k, T2R;
					     T1h = FMA(T1f, T1g, T1d);
					     T3z = T1b * T1g;
					     T1k = T2 * T1j;
					     T2R = T2 * T1l;
					     T1i = T19 + T1h;
					     T2Q = T19 - T1h;
					     T3A = FNMS(T1f, T1c, T3z);
					     T1m = FMA(T5, T1l, T1k);
					     T2S = FNMS(T5, T1j, T2R);
					}
					TE = Te - TD;
					T2u = Te + TD;
					{
					     E T4e, T3B, T1x, T3w;
					     T4e = T3y + T3A;
					     T3B = T3y - T3A;
					     T1x = T1m + T1w;
					     T3w = T1w - T1m;
					     {
						  E T4f, T2V, T1y, T4y;
						  T4f = T2S + T2U;
						  T2V = T2S - T2U;
						  T3C = T3w - T3B;
						  T40 = T3B + T3w;
						  T1y = T1i - T1x;
						  T2w = T1i + T1x;
						  T2W = T2Q - T2V;
						  T3Q = T2Q + T2V;
						  T4g = T4e + T4f;
						  T4y = T4f - T4e;
						  T53 = T1y - T11;
						  T1z = T11 + T1y;
						  T2s = T20 + T2r;
						  T52 = T20 - T2r;
						  T4V = T4z + T4y;
						  T4A = T4y - T4z;
					     }
					}
				   }
				   T4t = T1z - T2s;
				   T2t = T1z + T2s;
				   T4D = FMA(KP618033988, T4x, T4A);
				   T4B = FNMS(KP618033988, T4A, T4x);
				   T54 = FMA(KP618033988, T53, T52);
				   T56 = FNMS(KP618033988, T52, T53);
				   Rm[WS(rs, 9)] = TE + T2t;
				   T4s = FNMS(KP250000000, T2t, TE);
				   T4Y = T4L - T4H;
				   T4M = T4H + T4L;
				   T4u = FNMS(KP559016994, T4t, T4s);
				   T4C = FMA(KP559016994, T4t, T4s);
			      }
			      {
				   E T2x, T4Q, T4p, T4r, T4R, T2A, T51, T55;
				   {
					E T4h, T50, T4X, T4o, T4Z;
					T4E = T4d + T4g;
					T4h = T4d - T4g;
					Rm[WS(rs, 1)] = FMA(KP951056516, T4B, T4u);
					Rp[WS(rs, 2)] = FNMS(KP951056516, T4B, T4u);
					Rp[WS(rs, 6)] = FMA(KP951056516, T4D, T4C);
					Rm[WS(rs, 5)] = FNMS(KP951056516, T4D, T4C);
					T50 = T4W - T4V;
					T4X = T4V + T4W;
					T4o = T4k - T4n;
					T4F = T4k + T4n;
					T2x = T2v + T2w;
					T4Q = T2v - T2w;
					Im[WS(rs, 9)] = T4X - T4Y;
					T4Z = FMA(KP250000000, T4X, T4Y);
					T4p = FMA(KP618033988, T4o, T4h);
					T4r = FNMS(KP618033988, T4h, T4o);
					T4R = T2z - T2y;
					T2A = T2y + T2z;
					T51 = FNMS(KP559016994, T50, T4Z);
					T55 = FMA(KP559016994, T50, T4Z);
				   }
				   {
					E T49, T48, T2B, T4a, T4q;
					T2B = T2x + T2A;
					T49 = T2x - T2A;
					Ip[WS(rs, 2)] = FMA(KP951056516, T54, T51);
					Im[WS(rs, 1)] = FMS(KP951056516, T54, T51);
					Ip[WS(rs, 6)] = FMA(KP951056516, T56, T55);
					Im[WS(rs, 5)] = FMS(KP951056516, T56, T55);
					Rp[0] = T2u + T2B;
					T48 = FNMS(KP250000000, T2B, T2u);
					T4a = FMA(KP559016994, T49, T48);
					T4q = FNMS(KP559016994, T49, T48);
					T4U = FMA(KP618033988, T4Q, T4R);
					T4S = FNMS(KP618033988, T4R, T4Q);
					Rm[WS(rs, 3)] = FMA(KP951056516, T4p, T4a);
					Rp[WS(rs, 4)] = FNMS(KP951056516, T4p, T4a);
					Rp[WS(rs, 8)] = FMA(KP951056516, T4r, T4q);
					Rm[WS(rs, 7)] = FNMS(KP951056516, T4r, T4q);
				   }
			      }
			 }
			 {
			      E T3O, T5u, T5w, T5o, T5q, T5n;
			      {
				   E T5m, T5l, T2I, T4O, T3N, T3L, T2X, T5s, T4N, T5t, T3c, T3v, T3K, T4G;
				   T5m = T3u + T3n;
				   T3v = T3n - T3u;
				   T3K = T3C - T3J;
				   T5l = T3J + T3C;
				   T3O = T2C + T2H;
				   T2I = T2C - T2H;
				   T4O = T4E - T4F;
				   T4G = T4E + T4F;
				   T3N = FMA(KP618033988, T3v, T3K);
				   T3L = FNMS(KP618033988, T3K, T3v);
				   T2X = T2P + T2W;
				   T5s = T2P - T2W;
				   Ip[0] = T4G + T4M;
				   T4N = FNMS(KP250000000, T4G, T4M);
				   T5t = T34 - T3b;
				   T3c = T34 + T3b;
				   {
					E T3f, T3e, T4P, T4T, T3d, T3M, T3g;
					T4P = FMA(KP559016994, T4O, T4N);
					T4T = FNMS(KP559016994, T4O, T4N);
					T3f = T2X - T3c;
					T3d = T2X + T3c;
					Ip[WS(rs, 4)] = FMA(KP951056516, T4S, T4P);
					Im[WS(rs, 3)] = FMS(KP951056516, T4S, T4P);
					Ip[WS(rs, 8)] = FMA(KP951056516, T4U, T4T);
					Im[WS(rs, 7)] = FMS(KP951056516, T4U, T4T);
					Rm[WS(rs, 4)] = T2I + T3d;
					T3e = FNMS(KP250000000, T3d, T2I);
					T5u = FMA(KP618033988, T5t, T5s);
					T5w = FNMS(KP618033988, T5s, T5t);
					T5o = T58 + T57;
					T59 = T57 - T58;
					T3M = FMA(KP559016994, T3f, T3e);
					T3g = FNMS(KP559016994, T3f, T3e);
					Rp[WS(rs, 7)] = FNMS(KP951056516, T3L, T3g);
					Rp[WS(rs, 3)] = FMA(KP951056516, T3L, T3g);
					Rm[0] = FNMS(KP951056516, T3N, T3M);
					Rm[WS(rs, 8)] = FMA(KP951056516, T3N, T3M);
					T5q = T5l - T5m;
					T5n = T5l + T5m;
				   }
			      }
			      {
				   E T5a, T5b, T47, T45, T5h, T5g, T3V, T3X, T41, T44, T5p, T3W, T46, T3Y;
				   T5a = T3Z + T40;
				   T41 = T3Z - T40;
				   T44 = T42 - T43;
				   T5b = T42 + T43;
				   Im[WS(rs, 4)] = T5n - T5o;
				   T5p = FMA(KP250000000, T5n, T5o);
				   T47 = FNMS(KP618033988, T41, T44);
				   T45 = FMA(KP618033988, T44, T41);
				   {
					E T5r, T5v, T3R, T3U;
					T5r = FNMS(KP559016994, T5q, T5p);
					T5v = FMA(KP559016994, T5q, T5p);
					T3R = T3P + T3Q;
					T5h = T3P - T3Q;
					T5g = T3S - T3T;
					T3U = T3S + T3T;
					Im[0] = -(FMA(KP951056516, T5u, T5r));
					Im[WS(rs, 8)] = FMS(KP951056516, T5u, T5r);
					Ip[WS(rs, 7)] = FMA(KP951056516, T5w, T5v);
					Ip[WS(rs, 3)] = FNMS(KP951056516, T5w, T5v);
					T3V = T3R + T3U;
					T3X = T3R - T3U;
				   }
				   Rp[WS(rs, 5)] = T3O + T3V;
				   T3W = FNMS(KP250000000, T3V, T3O);
				   T5i = FNMS(KP618033988, T5h, T5g);
				   T5k = FMA(KP618033988, T5g, T5h);
				   T46 = FNMS(KP559016994, T3X, T3W);
				   T3Y = FMA(KP559016994, T3X, T3W);
				   Rp[WS(rs, 9)] = FNMS(KP951056516, T45, T3Y);
				   Rp[WS(rs, 1)] = FMA(KP951056516, T45, T3Y);
				   Rm[WS(rs, 2)] = FNMS(KP951056516, T47, T46);
				   Rm[WS(rs, 6)] = FMA(KP951056516, T47, T46);
				   T5e = T5a - T5b;
				   T5c = T5a + T5b;
			      }
			 }
		    }
	       }
	  }
	  Ip[WS(rs, 5)] = T5c + T59;
	  T5d = FNMS(KP250000000, T5c, T59);
	  T5j = FMA(KP559016994, T5e, T5d);
	  T5f = FNMS(KP559016994, T5e, T5d);
	  Im[WS(rs, 2)] = -(FMA(KP951056516, T5i, T5f));
	  Im[WS(rs, 6)] = FMS(KP951056516, T5i, T5f);
	  Ip[WS(rs, 9)] = FMA(KP951056516, T5k, T5j);
	  Ip[WS(rs, 1)] = FNMS(KP951056516, T5k, T5j);
     }
}

static const tw_instr twinstr[] = {
     {TW_CEXP, 1, 1},
     {TW_CEXP, 1, 3},
     {TW_CEXP, 1, 9},
     {TW_CEXP, 1, 19},
     {TW_NEXT, 1, 0}
};

static const hc2c_desc desc = { 20, "hc2cf2_20", twinstr, &GENUS, {136, 58, 140, 0} };

void X(codelet_hc2cf2_20) (planner *p) {
     X(khc2c_register) (p, hc2cf2_20, &desc, HC2C_VIA_RDFT);
}
#else				/* HAVE_FMA */

/* Generated by: ../../../genfft/gen_hc2c -compact -variables 4 -pipeline-latency 4 -twiddle-log3 -precompute-twiddles -n 20 -dit -name hc2cf2_20 -include hc2cf.h */

/*
 * This function contains 276 FP additions, 164 FP multiplications,
 * (or, 204 additions, 92 multiplications, 72 fused multiply/add),
 * 123 stack variables, 4 constants, and 80 memory accesses
 */
#include "hc2cf.h"

static void hc2cf2_20(R *Rp, R *Ip, R *Rm, R *Im, const R *W, stride rs, INT mb, INT me, INT ms)
{
     DK(KP587785252, +0.587785252292473129168705954639072768597652438);
     DK(KP951056516, +0.951056516295153572116439333379382143405698634);
     DK(KP250000000, +0.250000000000000000000000000000000000000000000);
     DK(KP559016994, +0.559016994374947424102293417182819058860154590);
     INT m;
     for (m = mb, W = W + ((mb - 1) * 8); m < me; m = m + 1, Rp = Rp + ms, Ip = Ip + ms, Rm = Rm - ms, Im = Im - ms, W = W + 8, MAKE_VOLATILE_STRIDE(rs)) {
	  E T2, T5, Tg, Ti, Tk, To, T1h, T1f, T6, T3, T8, T14, T1Q, Tc, T1O;
	  E T1v, T18, T1t, T1n, T24, T1j, T22, Tq, Tu, T1E, T1G, Tx, Ty, Tz, TJ;
	  E T1Z, TB, T1X, T1A, TZ, TL, T1y, TX;
	  {
	       E T7, T16, Ta, T13, T4, T17, Tb, T12;
	       {
		    E Th, Tn, Tj, Tm;
		    T2 = W[0];
		    T5 = W[1];
		    Tg = W[2];
		    Ti = W[3];
		    Th = T2 * Tg;
		    Tn = T5 * Tg;
		    Tj = T5 * Ti;
		    Tm = T2 * Ti;
		    Tk = Th - Tj;
		    To = Tm + Tn;
		    T1h = Tm - Tn;
		    T1f = Th + Tj;
		    T6 = W[5];
		    T7 = T5 * T6;
		    T16 = Tg * T6;
		    Ta = T2 * T6;
		    T13 = Ti * T6;
		    T3 = W[4];
		    T4 = T2 * T3;
		    T17 = Ti * T3;
		    Tb = T5 * T3;
		    T12 = Tg * T3;
	       }
	       T8 = T4 - T7;
	       T14 = T12 + T13;
	       T1Q = T16 + T17;
	       Tc = Ta + Tb;
	       T1O = T12 - T13;
	       T1v = Ta - Tb;
	       T18 = T16 - T17;
	       T1t = T4 + T7;
	       {
		    E T1l, T1m, T1g, T1i;
		    T1l = T1f * T6;
		    T1m = T1h * T3;
		    T1n = T1l + T1m;
		    T24 = T1l - T1m;
		    T1g = T1f * T3;
		    T1i = T1h * T6;
		    T1j = T1g - T1i;
		    T22 = T1g + T1i;
		    {
			 E Tl, Tp, Ts, Tt;
			 Tl = Tk * T3;
			 Tp = To * T6;
			 Tq = Tl + Tp;
			 Ts = Tk * T6;
			 Tt = To * T3;
			 Tu = Ts - Tt;
			 T1E = Tl - Tp;
			 T1G = Ts + Tt;
			 Tx = W[6];
			 Ty = W[7];
			 Tz = FMA(Tk, Tx, To * Ty);
			 TJ = FMA(Tq, Tx, Tu * Ty);
			 T1Z = FNMS(T1h, Tx, T1f * Ty);
			 TB = FNMS(To, Tx, Tk * Ty);
			 T1X = FMA(T1f, Tx, T1h * Ty);
			 T1A = FNMS(T5, Tx, T2 * Ty);
			 TZ = FNMS(Ti, Tx, Tg * Ty);
			 TL = FNMS(Tu, Tx, Tq * Ty);
			 T1y = FMA(T2, Tx, T5 * Ty);
			 TX = FMA(Tg, Tx, Ti * Ty);
		    }
	       }
	  }
	  {
	       E TF, T2b, T4D, T4M, T2K, T3r, T4a, T4m, T1N, T28, T29, T3J, T3M, T44, T3U;
	       E T3V, T4j, T2f, T2g, T2h, T2n, T2s, T4K, T3g, T3h, T4z, T3n, T3o, T3p, T30;
	       E T35, T36, TW, T1r, T1s, T3C, T3F, T43, T3X, T3Y, T4k, T2c, T2d, T2e, T2y;
	       E T2D, T4J, T3d, T3e, T4y, T3k, T3l, T3m, T2P, T2U, T2V;
	       {
		    E T1, T48, Te, T47, Tw, T2H, TD, T2I, T9, Td;
		    T1 = Rp[0];
		    T48 = Rm[0];
		    T9 = Rp[WS(rs, 5)];
		    Td = Rm[WS(rs, 5)];
		    Te = FMA(T8, T9, Tc * Td);
		    T47 = FNMS(Tc, T9, T8 * Td);
		    {
			 E Tr, Tv, TA, TC;
			 Tr = Ip[WS(rs, 2)];
			 Tv = Im[WS(rs, 2)];
			 Tw = FMA(Tq, Tr, Tu * Tv);
			 T2H = FNMS(Tu, Tr, Tq * Tv);
			 TA = Ip[WS(rs, 7)];
			 TC = Im[WS(rs, 7)];
			 TD = FMA(Tz, TA, TB * TC);
			 T2I = FNMS(TB, TA, Tz * TC);
		    }
		    {
			 E Tf, TE, T4B, T4C;
			 Tf = T1 + Te;
			 TE = Tw + TD;
			 TF = Tf - TE;
			 T2b = Tf + TE;
			 T4B = T48 - T47;
			 T4C = Tw - TD;
			 T4D = T4B - T4C;
			 T4M = T4C + T4B;
		    }
		    {
			 E T2G, T2J, T46, T49;
			 T2G = T1 - Te;
			 T2J = T2H - T2I;
			 T2K = T2G - T2J;
			 T3r = T2G + T2J;
			 T46 = T2H + T2I;
			 T49 = T47 + T48;
			 T4a = T46 + T49;
			 T4m = T49 - T46;
		    }
	       }
	       {
		    E T1D, T3H, T2l, T2W, T27, T3L, T2r, T34, T1M, T3I, T2m, T2Z, T1W, T3K, T2q;
		    E T31;
		    {
			 E T1x, T2j, T1C, T2k;
			 {
			      E T1u, T1w, T1z, T1B;
			      T1u = Rp[WS(rs, 4)];
			      T1w = Rm[WS(rs, 4)];
			      T1x = FMA(T1t, T1u, T1v * T1w);
			      T2j = FNMS(T1v, T1u, T1t * T1w);
			      T1z = Rp[WS(rs, 9)];
			      T1B = Rm[WS(rs, 9)];
			      T1C = FMA(T1y, T1z, T1A * T1B);
			      T2k = FNMS(T1A, T1z, T1y * T1B);
			 }
			 T1D = T1x + T1C;
			 T3H = T2j + T2k;
			 T2l = T2j - T2k;
			 T2W = T1x - T1C;
		    }
		    {
			 E T21, T32, T26, T33;
			 {
			      E T1Y, T20, T23, T25;
			      T1Y = Ip[WS(rs, 8)];
			      T20 = Im[WS(rs, 8)];
			      T21 = FMA(T1X, T1Y, T1Z * T20);
			      T32 = FNMS(T1Z, T1Y, T1X * T20);
			      T23 = Ip[WS(rs, 3)];
			      T25 = Im[WS(rs, 3)];
			      T26 = FMA(T22, T23, T24 * T25);
			      T33 = FNMS(T24, T23, T22 * T25);
			 }
			 T27 = T21 + T26;
			 T3L = T32 + T33;
			 T2r = T21 - T26;
			 T34 = T32 - T33;
		    }
		    {
			 E T1I, T2X, T1L, T2Y;
			 {
			      E T1F, T1H, T1J, T1K;
			      T1F = Ip[WS(rs, 6)];
			      T1H = Im[WS(rs, 6)];
			      T1I = FMA(T1E, T1F, T1G * T1H);
			      T2X = FNMS(T1G, T1F, T1E * T1H);
			      T1J = Ip[WS(rs, 1)];
			      T1K = Im[WS(rs, 1)];
			      T1L = FMA(Tg, T1J, Ti * T1K);
			      T2Y = FNMS(Ti, T1J, Tg * T1K);
			 }
			 T1M = T1I + T1L;
			 T3I = T2X + T2Y;
			 T2m = T1I - T1L;
			 T2Z = T2X - T2Y;
		    }
		    {
			 E T1S, T2o, T1V, T2p;
			 {
			      E T1P, T1R, T1T, T1U;
			      T1P = Rp[WS(rs, 6)];
			      T1R = Rm[WS(rs, 6)];
			      T1S = FMA(T1O, T1P, T1Q * T1R);
			      T2o = FNMS(T1Q, T1P, T1O * T1R);
			      T1T = Rp[WS(rs, 1)];
			      T1U = Rm[WS(rs, 1)];
			      T1V = FMA(T1f, T1T, T1h * T1U);
			      T2p = FNMS(T1h, T1T, T1f * T1U);
			 }
			 T1W = T1S + T1V;
			 T3K = T2o + T2p;
			 T2q = T2o - T2p;
			 T31 = T1S - T1V;
		    }
		    T1N = T1D - T1M;
		    T28 = T1W - T27;
		    T29 = T1N + T28;
		    T3J = T3H + T3I;
		    T3M = T3K + T3L;
		    T44 = T3J + T3M;
		    T3U = T3H - T3I;
		    T3V = T3L - T3K;
		    T4j = T3V - T3U;
		    T2f = T1D + T1M;
		    T2g = T1W + T27;
		    T2h = T2f + T2g;
		    T2n = T2l + T2m;
		    T2s = T2q + T2r;
		    T4K = T2n + T2s;
		    T3g = T2l - T2m;
		    T3h = T2q - T2r;
		    T4z = T3g + T3h;
		    T3n = T2W + T2Z;
		    T3o = T31 + T34;
		    T3p = T3n + T3o;
		    T30 = T2W - T2Z;
		    T35 = T31 - T34;
		    T36 = T30 + T35;
	       }
	       {
		    E TO, T3A, T2w, T2L, T1q, T3E, T2z, T2T, TV, T3B, T2x, T2O, T1b, T3D, T2C;
		    E T2Q;
		    {
			 E TI, T2u, TN, T2v;
			 {
			      E TG, TH, TK, TM;
			      TG = Rp[WS(rs, 2)];
			      TH = Rm[WS(rs, 2)];
			      TI = FMA(Tk, TG, To * TH);
			      T2u = FNMS(To, TG, Tk * TH);
			      TK = Rp[WS(rs, 7)];
			      TM = Rm[WS(rs, 7)];
			      TN = FMA(TJ, TK, TL * TM);
			      T2v = FNMS(TL, TK, TJ * TM);
			 }
			 TO = TI + TN;
			 T3A = T2u + T2v;
			 T2w = T2u - T2v;
			 T2L = TI - TN;
		    }
		    {
			 E T1e, T2R, T1p, T2S;
			 {
			      E T1c, T1d, T1k, T1o;
			      T1c = Ip[0];
			      T1d = Im[0];
			      T1e = FMA(T2, T1c, T5 * T1d);
			      T2R = FNMS(T5, T1c, T2 * T1d);
			      T1k = Ip[WS(rs, 5)];
			      T1o = Im[WS(rs, 5)];
			      T1p = FMA(T1j, T1k, T1n * T1o);
			      T2S = FNMS(T1n, T1k, T1j * T1o);
			 }
			 T1q = T1e + T1p;
			 T3E = T2R + T2S;
			 T2z = T1p - T1e;
			 T2T = T2R - T2S;
		    }
		    {
			 E TR, T2M, TU, T2N;
			 {
			      E TP, TQ, TS, TT;
			      TP = Ip[WS(rs, 4)];
			      TQ = Im[WS(rs, 4)];
			      TR = FMA(T3, TP, T6 * TQ);
			      T2M = FNMS(T6, TP, T3 * TQ);
			      TS = Ip[WS(rs, 9)];
			      TT = Im[WS(rs, 9)];
			      TU = FMA(Tx, TS, Ty * TT);
			      T2N = FNMS(Ty, TS, Tx * TT);
			 }
			 TV = TR + TU;
			 T3B = T2M + T2N;
			 T2x = TR - TU;
			 T2O = T2M - T2N;
		    }
		    {
			 E T11, T2A, T1a, T2B;
			 {
			      E TY, T10, T15, T19;
			      TY = Rp[WS(rs, 8)];
			      T10 = Rm[WS(rs, 8)];
			      T11 = FMA(TX, TY, TZ * T10);
			      T2A = FNMS(TZ, TY, TX * T10);
			      T15 = Rp[WS(rs, 3)];
			      T19 = Rm[WS(rs, 3)];
			      T1a = FMA(T14, T15, T18 * T19);
			      T2B = FNMS(T18, T15, T14 * T19);
			 }
			 T1b = T11 + T1a;
			 T3D = T2A + T2B;
			 T2C = T2A - T2B;
			 T2Q = T11 - T1a;
		    }
		    TW = TO - TV;
		    T1r = T1b - T1q;
		    T1s = TW + T1r;
		    T3C = T3A + T3B;
		    T3F = T3D + T3E;
		    T43 = T3C + T3F;
		    T3X = T3A - T3B;
		    T3Y = T3D - T3E;
		    T4k = T3X + T3Y;
		    T2c = TO + TV;
		    T2d = T1b + T1q;
		    T2e = T2c + T2d;
		    T2y = T2w + T2x;
		    T2D = T2z - T2C;
		    T4J = T2D - T2y;
		    T3d = T2w - T2x;
		    T3e = T2C + T2z;
		    T4y = T3d + T3e;
		    T3k = T2L + T2O;
		    T3l = T2Q + T2T;
		    T3m = T3k + T3l;
		    T2P = T2L - T2O;
		    T2U = T2Q - T2T;
		    T2V = T2P + T2U;
	       }
	       {
		    E T3S, T2a, T3R, T40, T42, T3W, T3Z, T41, T3T;
		    T3S = KP559016994 * (T1s - T29);
		    T2a = T1s + T29;
		    T3R = FNMS(KP250000000, T2a, TF);
		    T3W = T3U + T3V;
		    T3Z = T3X - T3Y;
		    T40 = FNMS(KP587785252, T3Z, KP951056516 * T3W);
		    T42 = FMA(KP951056516, T3Z, KP587785252 * T3W);
		    Rm[WS(rs, 9)] = TF + T2a;
		    T41 = T3S + T3R;
		    Rm[WS(rs, 5)] = T41 - T42;
		    Rp[WS(rs, 6)] = T41 + T42;
		    T3T = T3R - T3S;
		    Rp[WS(rs, 2)] = T3T - T40;
		    Rm[WS(rs, 1)] = T3T + T40;
	       }
	       {
		    E T4r, T4l, T4q, T4p, T4t, T4n, T4o, T4u, T4s;
		    T4r = KP559016994 * (T4k + T4j);
		    T4l = T4j - T4k;
		    T4q = FMA(KP250000000, T4l, T4m);
		    T4n = T1r - TW;
		    T4o = T1N - T28;
		    T4p = FMA(KP587785252, T4n, KP951056516 * T4o);
		    T4t = FNMS(KP587785252, T4o, KP951056516 * T4n);
		    Im[WS(rs, 9)] = T4l - T4m;
		    T4u = T4r + T4q;
		    Im[WS(rs, 5)] = T4t - T4u;
		    Ip[WS(rs, 6)] = T4t + T4u;
		    T4s = T4q - T4r;
		    Im[WS(rs, 1)] = T4p - T4s;
		    Ip[WS(rs, 2)] = T4p + T4s;
	       }
	       {
		    E T3x, T2i, T3y, T3O, T3Q, T3G, T3N, T3P, T3z;
		    T3x = KP559016994 * (T2e - T2h);
		    T2i = T2e + T2h;
		    T3y = FNMS(KP250000000, T2i, T2b);
		    T3G = T3C - T3F;
		    T3N = T3J - T3M;
		    T3O = FMA(KP951056516, T3G, KP587785252 * T3N);
		    T3Q = FNMS(KP587785252, T3G, KP951056516 * T3N);
		    Rp[0] = T2b + T2i;
		    T3P = T3y - T3x;
		    Rm[WS(rs, 7)] = T3P - T3Q;
		    Rp[WS(rs, 8)] = T3P + T3Q;
		    T3z = T3x + T3y;
		    Rp[WS(rs, 4)] = T3z - T3O;
		    Rm[WS(rs, 3)] = T3z + T3O;
	       }
	       {
		    E T4e, T45, T4f, T4d, T4h, T4b, T4c, T4i, T4g;
		    T4e = KP559016994 * (T43 - T44);
		    T45 = T43 + T44;
		    T4f = FNMS(KP250000000, T45, T4a);
		    T4b = T2c - T2d;
		    T4c = T2f - T2g;
		    T4d = FMA(KP951056516, T4b, KP587785252 * T4c);
		    T4h = FNMS(KP951056516, T4c, KP587785252 * T4b);
		    Ip[0] = T45 + T4a;
		    T4i = T4f - T4e;
		    Im[WS(rs, 7)] = T4h - T4i;
		    Ip[WS(rs, 8)] = T4h + T4i;
		    T4g = T4e + T4f;
		    Im[WS(rs, 3)] = T4d - T4g;
		    Ip[WS(rs, 4)] = T4d + T4g;
	       }
	       {
		    E T39, T37, T38, T2F, T3b, T2t, T2E, T3c, T3a;
		    T39 = KP559016994 * (T2V - T36);
		    T37 = T2V + T36;
		    T38 = FNMS(KP250000000, T37, T2K);
		    T2t = T2n - T2s;
		    T2E = T2y + T2D;
		    T2F = FNMS(KP587785252, T2E, KP951056516 * T2t);
		    T3b = FMA(KP951056516, T2E, KP587785252 * T2t);
		    Rm[WS(rs, 4)] = T2K + T37;
		    T3c = T39 + T38;
		    Rm[WS(rs, 8)] = T3b + T3c;
		    Rm[0] = T3c - T3b;
		    T3a = T38 - T39;
		    Rp[WS(rs, 3)] = T2F + T3a;
		    Rp[WS(rs, 7)] = T3a - T2F;
	       }
	       {
		    E T4Q, T4L, T4R, T4P, T4U, T4N, T4O, T4T, T4S;
		    T4Q = KP559016994 * (T4J + T4K);
		    T4L = T4J - T4K;
		    T4R = FMA(KP250000000, T4L, T4M);
		    T4N = T2P - T2U;
		    T4O = T30 - T35;
		    T4P = FMA(KP951056516, T4N, KP587785252 * T4O);
		    T4U = FNMS(KP587785252, T4N, KP951056516 * T4O);
		    Im[WS(rs, 4)] = T4L - T4M;
		    T4T = T4Q + T4R;
		    Ip[WS(rs, 3)] = T4T - T4U;
		    Ip[WS(rs, 7)] = T4U + T4T;
		    T4S = T4Q - T4R;
		    Im[WS(rs, 8)] = T4P + T4S;
		    Im[0] = T4S - T4P;
	       }
	       {
		    E T3q, T3s, T3t, T3j, T3v, T3f, T3i, T3w, T3u;
		    T3q = KP559016994 * (T3m - T3p);
		    T3s = T3m + T3p;
		    T3t = FNMS(KP250000000, T3s, T3r);
		    T3f = T3d - T3e;
		    T3i = T3g - T3h;
		    T3j = FMA(KP951056516, T3f, KP587785252 * T3i);
		    T3v = FNMS(KP587785252, T3f, KP951056516 * T3i);
		    Rp[WS(rs, 5)] = T3r + T3s;
		    T3w = T3t - T3q;
		    Rm[WS(rs, 6)] = T3v + T3w;
		    Rm[WS(rs, 2)] = T3w - T3v;
		    T3u = T3q + T3t;
		    Rp[WS(rs, 1)] = T3j + T3u;
		    Rp[WS(rs, 9)] = T3u - T3j;
	       }
	       {
		    E T4A, T4E, T4F, T4x, T4I, T4v, T4w, T4H, T4G;
		    T4A = KP559016994 * (T4y - T4z);
		    T4E = T4y + T4z;
		    T4F = FNMS(KP250000000, T4E, T4D);
		    T4v = T3n - T3o;
		    T4w = T3k - T3l;
		    T4x = FNMS(KP587785252, T4w, KP951056516 * T4v);
		    T4I = FMA(KP951056516, T4w, KP587785252 * T4v);
		    Ip[WS(rs, 5)] = T4E + T4D;
		    T4H = T4A + T4F;
		    Ip[WS(rs, 1)] = T4H - T4I;
		    Ip[WS(rs, 9)] = T4I + T4H;
		    T4G = T4A - T4F;
		    Im[WS(rs, 6)] = T4x + T4G;
		    Im[WS(rs, 2)] = T4G - T4x;
	       }
	  }
     }
}

static const tw_instr twinstr[] = {
     {TW_CEXP, 1, 1},
     {TW_CEXP, 1, 3},
     {TW_CEXP, 1, 9},
     {TW_CEXP, 1, 19},
     {TW_NEXT, 1, 0}
};

static const hc2c_desc desc = { 20, "hc2cf2_20", twinstr, &GENUS, {204, 92, 72, 0} };

void X(codelet_hc2cf2_20) (planner *p) {
     X(khc2c_register) (p, hc2cf2_20, &desc, HC2C_VIA_RDFT);
}
#endif				/* HAVE_FMA */
