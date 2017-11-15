/* Copyright (c) 2012-2014, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#define pr_fmt(fmt) "%s: " fmt, __func__

#include <linux/kernel.h>
#include <linux/of.h>
#include <linux/err.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/mutex.h>
#include <linux/types.h>
#include <linux/hwmon.h>
#include <linux/module.h>
#include <linux/debugfs.h>
#include <linux/spmi.h>
#include <linux/of_irq.h>
#include <linux/interrupt.h>
#include <linux/completion.h>
#include <linux/qpnp/qpnp-adc.h>
#include <linux/platform_device.h>

/* Min ADC code represets 0V */
#define QPNP_VADC_MIN_ADC_CODE			0x6000
/* Max ADC code represents full-scale range of 1.8V */
#define QPNP_VADC_MAX_ADC_CODE			0xA800
#define KELVINMIL_DEGMIL	273160

/* Units for temperature below (on x axis) is in 0.1DegC as
   required by the battery driver. Note the resolution used
   here to compute the table was done for DegC to milli-volts.
   In consideration to limit the size of the table for the given
   temperature range below, the result is linearly interpolated
   and provided to the battery driver in the units desired for
   their framework which is 0.1DegC. True resolution of 0.1DegC
   will result in the below table size to increase by 10 times */

/* [PLATFORM]-Mod-BEGIN by TCTNB.FLF, PR-833642, 2014/11/10, adjust thermal adc according to HW */
/* [PLATFORM]-Mod-BEGIN by TCTNB.FLF, PR-942957, PR-942970, 2015/03/06, adjust thermal adc according to HW */
#if defined(CONFIG_TCT_8X16_ALTO45_LATAM_B28) || defined(CONFIG_TCT_8X16_ALTO5) || defined(CONFIG_TCT_8X16_ALTO5_PREMIUM)
static const struct qpnp_vadc_map_pt adcmap_btm_threshold[] = {
	{-300,    1642},
    {-200,    1544},
    {-100,    1414},
    {0,    1260},
    {10,    1244},
    {20,    1228},
    {30,    1212},
    {40,    1195},
    {50,    1179},
    {60,    1162},
    {70,    1146},
    {80,    1129},
    {90,    1113},
    {100,    1097},
    {110,    1080},
    {120,    1064},
    {130,    1048},
    {140,    1032},
    {150,    1016},
    {160,    1000},
    {170,    985},
    {180,    969},
    {190,    954},
    {200,    939},
    {210,    924},
    {220,    909},
    {230,    894},
    {240,    880},
    {250,    866},
    {260,    852},
    {270,    838},
    {280,    824},
    {290,    811},
    {300,    798},
    {310,    785},
    {320,    771},
    {330,    758},
    {340,    745},
    {350,    732},
    {360,    718},
    {370,    705},
    {380,    692},
    {390,    678},
    {400,    665},
    {410,    648},
    {420,    632},
    {430,    615},
    {440,    599},
    {450,    582},
    {460,    573},
    {470,    564},
    {480,    554},
    {490,    545},
    {500,    536},
    {510,    526},
    {520,    516},
    {530,    506},
    {540,    496},
    {550,    486},
    {560,    474},
    {570,    462},
    {580,    451},
    {590,    439},
    {600,    428},
};
/* [PLATFORM]-Mod-BEGIN by TCTNB.YuBin, 2014/12/18, adjust thermal adc according for ALTO4 BYD battery */
#elif defined(CONFIG_TCT_8X16_ALTO4)
static const struct qpnp_vadc_map_pt adcmap_btm_threshold_scud[] = {
	{-400,	1678},
	{-390,	1671},
	{-380,	1665},
	{-370,	1658},
	{-360,	1652},
	{-350,	1645},
	{-340,	1637},
	{-330,	1630},
	{-320,	1622},
	{-310,	1614},
	{-300,	1606},
	{-290,	1598},
	{-280,	1589},
	{-270,	1581},
	{-260,	1572},
	{-250,	1562},
	{-240,	1553},
	{-230,	1543},
	{-220,	1533},
	{-210,	1523},
	{-200,	1513},
	{-190,	1502},
	{-180,	1491},
	{-170,	1480},
	{-160,	1469},
	{-150,	1458},
	{-140,	1446},
	{-130,	1434},
	{-120,	1422},
	{-110,	1410},
	{-100,	1397},
	{-90,	1384},
	{-80,	1371},
	{-70,	1358},
	{-60,	1345},
	{-50,	1332},
	{-40,	1318},
	{-30,	1304},
	{-20,	1290},
	{-10,	1276},
	{0,	1262},
	{10,	1248},
	{20,	1233},
	{30,	1219},
	{40,	1204},
	{50,	1190},
	{60,	1175},
	{70,	1160},
	{80,	1146},
	{90,	1131},
	{100,	1117},
	{110,	1102},
	{120,	1088},
	{130,	1074},
	{140,	1059},
	{150,	1045},
	{160,	1031},
	{170,	1017},
	{180,	1003},
	{190,	989},
	{200,	976},
	{210,	962},
	{220,	949},
	{230,	936},
	{240,	923},
	{250,	910},
	{260,	898},
	{270,	885},
	{280,	873},
	{290,	861},
	{300,	849},
	{310,	838},
	{320,	826},
	{330,	815},
	{340,	804},
	{350,	793},
	{360,	782},
	{370,	772},
	{380,	762},
	{390,	752},
	{400,	742},
	{410,	733},
	{420,	723},
	{430,	714},
	{440,	705},
	{450,	697},
	{460,	688},
	{470,	680},
	{480,	672},
	{490,	664},
	{500,	656},
	{510,	649},
	{520,	641},
	{530,	634},
	{540,	627},
	{550,	620},
	{560,	614},
	{570,	607},
	{580,	601},
	{590,	595},
	{600,	589},
	{610,	583},
	{620,	577},
	{630,	572},
	{640,	566},
	{650,	561},
	{660,	556},
	{670,	551},
	{680,	546},
	{690,	542},
	{700,	537},
	{710,	533},
	{720,	528},
	{730,	524},
	{740,	520},
	{750,	516},
	{760,	512},
	{770,	508},
	{780,	505},
	{790,	501},
	{800,	498},
	{810,	494},
	{820,	491},
	{830,	488},
	{840,	485},
	{850,	482},
	{860,	479},
	{870,	476},
	{880,	473},
	{890,	471},
	{900,	468},
	{910,	466},
	{920,	463},
	{930,	461},
	{940,	458},
	{950,	456},
	{960,	454},
	{970,	452},
	{980,	450},
	{990,	448},
	{1000,	446},
	{1010,	444},
	{1020,	442},
	{1030,	440},
	{1040,	438},
	{1050,	437},
	{1060,	435},
	{1070,	433},
	{1080,	432},
	{1090,	430},
	{1100,	429},
	{1110,	427},
	{1120,	426},
	{1130,	424},
	{1140,	423},
	{1150,	422},
	{1160,	420},
	{1170,	419},
	{1180,	418},
	{1190,	417},
	{1200,	416},
	{1210,	415},
	{1220,	413},
	{1230,	412},
	{1240,	411},
	{1250,	410}
};

static const struct qpnp_vadc_map_pt adcmap_btm_threshold_byd[] = {
	{-400,	1694},
	{-390,	1688},
	{-380,	1682},
	{-370,	1676},
	{-360,	1669},
	{-350,	1662},
	{-340,	1655},
	{-330,	1648},
	{-320,	1641},
	{-310,	1633},
	{-300,	1625},
	{-290,	1617},
	{-280,	1608},
	{-270,	1599},
	{-260,	1590},
	{-250,	1581},
	{-240,	1571},
	{-230,	1561},
	{-220,	1551},
	{-210,	1540},
	{-200,	1530},
	{-190,	1519},
	{-180,	1507},
	{-170,	1496},
	{-160,	1484},
	{-150,	1472},
	{-140,	1460},
	{-130,	1448},
	{-120,	1435},
	{-110,	1422},
	{-100,	1409},
	{-90,	1396},
	{-80,	1382},
	{-70,	1369},
	{-60,	1355},
	{-50,	1341},
	{-40,	1327},
	{-30,	1312},
	{-20,	1298},
	{-10,	1284},
	{0,	1269},
	{10,	1254},
	{20,	1240},
	{30,	1225},
	{40,	1210},
	{50,	1196},
	{60,	1181},
	{70,	1166},
	{80,	1151},
	{90,	1137},
	{100,	1122},
	{110,	1107},
	{120,	1093},
	{130,	1078},
	{140,	1064},
	{150,	1049},
	{160,	1035},
	{170,	1021},
	{180,	1007},
	{190,	993},
	{200,	980},
	{210,	966},
	{220,	953},
	{230,	940},
	{240,	927},
	{250,	914},
	{260,	901},
	{270,	889},
	{280,	877},
	{290,	865},
	{300,	853},
	{310,	842},
	{320,	830},
	{330,	819},
	{340,	808},
	{350,	798},
	{360,	787},
	{370,	777},
	{380,	767},
	{390,	757},
	{400,	747},
	{410,	738},
	{420,	728},
	{430,	719},
	{440,	710},
	{450,	701},
	{460,	693},
	{470,	685},
	{480,	676},
	{490,	669},
	{500,	661},
	{510,	653},
	{520,	646},
	{530,	639},
	{540,	631},
	{550,	625},
	{560,	618},
	{570,	611},
	{580,	605},
	{590,	599},
	{600,	593},
	{610,	587},
	{620,	581},
	{630,	576},
	{640,	570},
	{650,	565},
	{660,	560},
	{670,	555},
	{680,	550},
	{690,	545},
	{700,	541},
	{710,	536},
	{720,	532},
	{730,	528},
	{740,	524},
	{750,	520},
	{760,	516},
	{770,	512},
	{780,	508},
	{790,	505},
	{800,	501},
	{810,	498},
	{820,	494},
	{830,	491},
	{840,	488},
	{850,	485},
	{860,	482},
	{870,	479},
	{880,	476},
	{890,	474},
	{900,	471},
	{910,	468},
	{920,	466},
	{930,	463},
	{940,	461},
	{950,	459},
	{960,	456},
	{970,	454},
	{980,	452},
	{990,	450},
	{1000,	448},
	{1010,	446},
	{1020,	444},
	{1030,	442},
	{1040,	440},
	{1050,	439},
	{1060,	437},
	{1070,	435},
	{1080,	434},
	{1090,	432},
	{1100,	430},
	{1110,	429},
	{1120,	427},
	{1130,	426},
	{1140,	425},
	{1150,	423},
	{1160,	422},
	{1170,	421},
	{1180,	419},
	{1190,	418},
	{1200,	417},
	{1210,	416},
	{1220,	415},
	{1230,	414},
	{1240,	413},
	{1250,	411}
};
/* [PLATFORM]-Mod-END by TCTNB.YuBin */
#else
static const struct qpnp_vadc_map_pt adcmap_btm_threshold[] = {
	{-300,	1642},
	{-200,	1544},
	{-100,	1414},
	{0,	1260},
	{10,	1244},
	{20,	1228},
	{30,	1212},
	{40,	1195},
	{50,	1179},
	{60,	1162},
	{70,	1146},
	{80,	1129},
	{90,	1113},
	{100,	1097},
	{110,	1080},
	{120,	1064},
	{130,	1048},
	{140,	1032},
	{150,	1016},
	{160,	1000},
	{170,	985},
	{180,	969},
	{190,	954},
	{200,	939},
	{210,	924},
	{220,	909},
	{230,	894},
	{240,	880},
	{250,	866},
	{260,	852},
	{270,	838},
	{280,	824},
	{290,	811},
	{300,	798},
	{310,	785},
	{320,	773},
	{330,	760},
	{340,	748},
	{350,	736},
	{360,	725},
	{370,	713},
	{380,	702},
	{390,	691},
	{400,	681},
	{410,	670},
	{420,	660},
	{430,	650},
	{440,	640},
	{450,	631},
	{460,	622},
	{470,	613},
	{480,	604},
	{490,	595},
	{500,	587},
	{510,	579},
	{520,	571},
	{530,	563},
	{540,	556},
	{550,	548},
	{560,	541},
	{570,	534},
	{580,	527},
	{590,	521},
	{600,	514},
	{610,	508},
	{620,	502},
	{630,	496},
	{640,	490},
	{650,	485},
	{660,	281},
	{670,	274},
	{680,	267},
	{690,	260},
	{700,	254},
	{710,	247},
	{720,	241},
	{730,	235},
	{740,	229},
	{750,	224},
	{760,	218},
	{770,	213},
	{780,	208},
	{790,	203}
};
#endif
/* [PLATFORM]-Mod-END by TCTNB.FLF */

static const struct qpnp_vadc_map_pt adcmap_qrd_btm_threshold[] = {
	{-200,	1540},
	{-180,	1517},
	{-160,	1492},
	{-140,	1467},
	{-120,	1440},
	{-100,	1412},
	{-80,	1383},
	{-60,	1353},
	{-40,	1323},
	{-20,	1292},
	{0,	1260},
	{20,	1228},
	{40,	1196},
	{60,	1163},
	{80,	1131},
	{100,	1098},
	{120,	1066},
	{140,	1034},
	{160,	1002},
	{180,	971},
	{200,	941},
	{220,	911},
	{240,	882},
	{260,	854},
	{280,	826},
	{300,	800},
	{320,	774},
	{340,	749},
	{360,	726},
	{380,	703},
	{400,	681},
	{420,	660},
	{440,	640},
	{460,	621},
	{480,	602},
	{500,	585},
	{520,	568},
	{540,	552},
	{560,	537},
	{580,	523},
	{600,	510},
	{620,	497},
	{640,	485},
	{660,	473},
	{680,	462},
	{700,	452},
	{720,	442},
	{740,	433},
	{760,	424},
	{780,	416},
	{800,	408},
};

static const struct qpnp_vadc_map_pt adcmap_qrd_skuaa_btm_threshold[] = {
	{-200,	1476},
	{-180,	1450},
	{-160,	1422},
	{-140,	1394},
	{-120,	1365},
	{-100,	1336},
	{-80,	1306},
	{-60,	1276},
	{-40,	1246},
	{-20,	1216},
	{0,	1185},
	{20,	1155},
	{40,	1126},
	{60,	1096},
	{80,	1068},
	{100,	1040},
	{120,	1012},
	{140,	986},
	{160,	960},
	{180,	935},
	{200,	911},
	{220,	888},
	{240,	866},
	{260,	844},
	{280,	824},
	{300,	805},
	{320,	786},
	{340,	769},
	{360,	752},
	{380,	737},
	{400,	722},
	{420,	707},
	{440,	694},
	{460,	681},
	{480,	669},
	{500,	658},
	{520,	648},
	{540,	637},
	{560,	628},
	{580,	619},
	{600,	611},
	{620,	603},
	{640,	595},
	{660,	588},
	{680,	582},
	{700,	575},
	{720,	569},
	{740,	564},
	{760,	559},
	{780,	554},
	{800,	549},
};

static const struct qpnp_vadc_map_pt adcmap_qrd_skug_btm_threshold[] = {
	{-200,	1338},
	{-180,	1307},
	{-160,	1276},
	{-140,	1244},
	{-120,	1213},
	{-100,	1182},
	{-80,	1151},
	{-60,	1121},
	{-40,	1092},
	{-20,	1063},
	{0,	1035},
	{20,	1008},
	{40,	982},
	{60,	957},
	{80,	933},
	{100,	910},
	{120,	889},
	{140,	868},
	{160,	848},
	{180,	830},
	{200,	812},
	{220,	795},
	{240,	780},
	{260,	765},
	{280,	751},
	{300,	738},
	{320,	726},
	{340,	714},
	{360,	704},
	{380,	694},
	{400,	684},
	{420,	675},
	{440,	667},
	{460,	659},
	{480,	652},
	{500,	645},
	{520,	639},
	{540,	633},
	{560,	627},
	{580,	622},
	{600,	617},
	{620,	613},
	{640,	608},
	{660,	604},
	{680,	600},
	{700,	597},
	{720,	593},
	{740,	590},
	{760,	587},
	{780,	585},
	{800,	582},
};

static const struct qpnp_vadc_map_pt adcmap_qrd_skuh_btm_threshold[] = {
	{-200,	1531},
	{-180,	1508},
	{-160,	1483},
	{-140,	1458},
	{-120,	1432},
	{-100,	1404},
	{-80,	1377},
	{-60,	1348},
	{-40,	1319},
	{-20,	1290},
	{0,	1260},
	{20,	1230},
	{40,	1200},
	{60,	1171},
	{80,	1141},
	{100,	1112},
	{120,	1083},
	{140,	1055},
	{160,	1027},
	{180,	1000},
	{200,	973},
	{220,	948},
	{240,	923},
	{260,	899},
	{280,	876},
	{300,	854},
	{320,	832},
	{340,	812},
	{360,	792},
	{380,	774},
	{400,	756},
	{420,	739},
	{440,	723},
	{460,	707},
	{480,	692},
	{500,	679},
	{520,	665},
	{540,	653},
	{560,	641},
	{580,	630},
	{600,	619},
	{620,	609},
	{640,	600},
	{660,	591},
	{680,	583},
	{700,	575},
	{720,	567},
	{740,	560},
	{760,	553},
	{780,	547},
	{800,	541},
	{820,	535},
	{840,	530},
	{860,	524},
	{880,	520},
};

/* Voltage to temperature */
static const struct qpnp_vadc_map_pt adcmap_100k_104ef_104fb[] = {
	{1758,	-40},
	{1742,	-35},
	{1719,	-30},
	{1691,	-25},
	{1654,	-20},
	{1608,	-15},
	{1551,	-10},
	{1483,	-5},
	{1404,	0},
	{1315,	5},
	{1218,	10},
	{1114,	15},
	{1007,	20},
	{900,	25},
	{795,	30},
	{696,	35},
	{605,	40},
	{522,	45},
	{448,	50},
	{383,	55},
	{327,	60},
	{278,	65},
	{237,	70},
	{202,	75},
	{172,	80},
	{146,	85},
	{125,	90},
	{107,	95},
	{92,	100},
	{79,	105},
	{68,	110},
	{59,	115},
	{51,	120},
	{44,	125}
};

/* Voltage to temperature */
static const struct qpnp_vadc_map_pt adcmap_150k_104ef_104fb[] = {
	{1738,	-40},
	{1714,	-35},
	{1682,	-30},
	{1641,	-25},
	{1589,	-20},
	{1526,	-15},
	{1451,	-10},
	{1363,	-5},
	{1266,	0},
	{1159,	5},
	{1048,	10},
	{936,	15},
	{825,	20},
	{720,	25},
	{622,	30},
	{533,	35},
	{454,	40},
	{385,	45},
	{326,	50},
	{275,	55},
	{232,	60},
	{195,	65},
	{165,	70},
	{139,	75},
	{118,	80},
	{100,	85},
	{85,	90},
	{73,	95},
	{62,	100},
	{53,	105},
	{46,	110},
	{40,	115},
	{34,	120},
	{30,	125}
};

static const struct qpnp_vadc_map_pt adcmap_smb_batt_therm[] = {
	{-300,	1625},
	{-200,	1515},
	{-100,	1368},
	{0,	1192},
	{10,	1173},
	{20,	1154},
	{30,	1135},
	{40,	1116},
	{50,	1097},
	{60,	1078},
	{70,	1059},
	{80,	1040},
	{90,	1020},
	{100,	1001},
	{110,	982},
	{120,	963},
	{130,	944},
	{140,	925},
	{150,	907},
	{160,	888},
	{170,	870},
	{180,	851},
	{190,	833},
	{200,	815},
	{210,	797},
	{220,	780},
	{230,	762},
	{240,	745},
	{250,	728},
	{260,	711},
	{270,	695},
	{280,	679},
	{290,	663},
	{300,	647},
	{310,	632},
	{320,	616},
	{330,	602},
	{340,	587},
	{350,	573},
	{360,	559},
	{370,	545},
	{380,	531},
	{390,	518},
	{400,	505},
	{410,	492},
	{420,	480},
	{430,	465},
	{440,	456},
	{450,	445},
	{460,	433},
	{470,	422},
	{480,	412},
	{490,	401},
	{500,	391},
	{510,	381},
	{520,	371},
	{530,	362},
	{540,	352},
	{550,	343},
	{560,	335},
	{570,	326},
	{580,	318},
	{590,	309},
	{600,	302},
	{610,	294},
	{620,	286},
	{630,	279},
	{640,	272},
	{650,	265},
	{660,	258},
	{670,	252},
	{680,	245},
	{690,	239},
	{700,	233},
	{710,	227},
	{720,	221},
	{730,	216},
	{740,	211},
	{750,	205},
	{760,	200},
	{770,	195},
	{780,	190},
	{790,	186}
};

/* Voltage to temperature */
static const struct qpnp_vadc_map_pt adcmap_ncp03wf683[] = {
	{1742,	-40},
	{1718,	-35},
	{1687,	-30},
	{1647,	-25},
	{1596,	-20},
	{1534,	-15},
	{1459,	-10},
	{1372,	-5},
	{1275,	0},
	{1169,	5},
	{1058,	10},
	{945,	15},
	{834,	20},
	{729,	25},
	{630,	30},
	{541,	35},
	{461,	40},
	{392,	45},
	{332,	50},
	{280,	55},
	{236,	60},
	{199,	65},
	{169,	70},
	{142,	75},
	{121,	80},
	{102,	85},
	{87,	90},
	{74,	95},
	{64,	100},
	{55,	105},
	{47,	110},
	{40,	115},
	{35,	120},
	{30,	125}
};

static int32_t qpnp_adc_map_voltage_temp(const struct qpnp_vadc_map_pt *pts,
		uint32_t tablesize, int32_t input, int64_t *output)
{
	bool descending = 1;
	uint32_t i = 0;

	if (pts == NULL)
		return -EINVAL;

	/* Check if table is descending or ascending */
	if (tablesize > 1) {
		if (pts[0].x < pts[1].x)
			descending = 0;
	}

	while (i < tablesize) {
		if ((descending == 1) && (pts[i].x < input)) {
			/* table entry is less than measured
				value and table is descending, stop */
			break;
		} else if ((descending == 0) &&
				(pts[i].x > input)) {
			/* table entry is greater than measured
				value and table is ascending, stop */
			break;
		} else {
			i++;
		}
	}

	if (i == 0)
		*output = pts[0].y;
	else if (i == tablesize)
		*output = pts[tablesize-1].y;
	else {
		/* result is between search_index and search_index-1 */
		/* interpolate linearly */
		*output = (((int32_t) ((pts[i].y - pts[i-1].y)*
			(input - pts[i-1].x))/
			(pts[i].x - pts[i-1].x))+
			pts[i-1].y);
	}

	return 0;
}

static int32_t qpnp_adc_map_temp_voltage(const struct qpnp_vadc_map_pt *pts,
		uint32_t tablesize, int32_t input, int64_t *output)
{
	bool descending = 1;
	uint32_t i = 0;

	if (pts == NULL)
		return -EINVAL;

	/* Check if table is descending or ascending */
	if (tablesize > 1) {
		if (pts[0].y < pts[1].y)
			descending = 0;
	}

	while (i < tablesize) {
		if ((descending == 1) && (pts[i].y < input)) {
			/* table entry is less than measured
				value and table is descending, stop */
			break;
		} else if ((descending == 0) && (pts[i].y > input)) {
			/* table entry is greater than measured
				value and table is ascending, stop */
			break;
		} else {
			i++;
		}
	}

	if (i == 0) {
		*output = pts[0].x;
	} else if (i == tablesize) {
		*output = pts[tablesize-1].x;
	} else {
		/* result is between search_index and search_index-1 */
		/* interpolate linearly */
		*output = (((int32_t) ((pts[i].x - pts[i-1].x)*
			(input - pts[i-1].y))/
			(pts[i].y - pts[i-1].y))+
			pts[i-1].x);
	}

	return 0;
}

static int64_t qpnp_adc_scale_ratiometric_calib(int32_t adc_code,
		const struct qpnp_adc_properties *adc_properties,
		const struct qpnp_vadc_chan_properties *chan_properties)
{
	int64_t adc_voltage = 0;
	bool negative_offset = 0;

	if (!chan_properties || !chan_properties->offset_gain_numerator ||
		!chan_properties->offset_gain_denominator || !adc_properties)
		return -EINVAL;

	adc_voltage = (adc_code -
		chan_properties->adc_graph[CALIB_RATIOMETRIC].adc_gnd)
		* adc_properties->adc_vdd_reference;
	if (adc_voltage < 0) {
		negative_offset = 1;
		adc_voltage = -adc_voltage;
	}
	do_div(adc_voltage,
		chan_properties->adc_graph[CALIB_RATIOMETRIC].dy);
	if (negative_offset)
		adc_voltage = -adc_voltage;

	return adc_voltage;
}

int32_t qpnp_adc_scale_pmic_therm(struct qpnp_vadc_chip *vadc,
		int32_t adc_code,
		const struct qpnp_adc_properties *adc_properties,
		const struct qpnp_vadc_chan_properties *chan_properties,
		struct qpnp_vadc_result *adc_chan_result)
{
	int64_t pmic_voltage = 0;
	bool negative_offset = 0;

	if (!chan_properties || !chan_properties->offset_gain_numerator ||
		!chan_properties->offset_gain_denominator || !adc_properties
		|| !adc_chan_result
		|| !chan_properties->adc_graph[CALIB_ABSOLUTE].dy)
		return -EINVAL;

	pmic_voltage = (adc_code -
		chan_properties->adc_graph[CALIB_ABSOLUTE].adc_gnd)
		* chan_properties->adc_graph[CALIB_ABSOLUTE].dx;
	if (pmic_voltage < 0) {
		negative_offset = 1;
		pmic_voltage = -pmic_voltage;
	}
	do_div(pmic_voltage,
		chan_properties->adc_graph[CALIB_ABSOLUTE].dy);
	if (negative_offset)
		pmic_voltage = -pmic_voltage;
	pmic_voltage += chan_properties->adc_graph[CALIB_ABSOLUTE].dx;

	if (pmic_voltage > 0) {
		/* 2mV/K */
		adc_chan_result->measurement = pmic_voltage*
			chan_properties->offset_gain_denominator;

		do_div(adc_chan_result->measurement,
			chan_properties->offset_gain_numerator * 2);
	} else {
		adc_chan_result->measurement = 0;
	}
	/* Change to .001 deg C */
	adc_chan_result->measurement -= KELVINMIL_DEGMIL;
	adc_chan_result->physical = (int32_t)adc_chan_result->measurement;

	return 0;
}
EXPORT_SYMBOL(qpnp_adc_scale_pmic_therm);

int32_t qpnp_adc_scale_millidegc_pmic_voltage_thr(struct qpnp_vadc_chip *chip,
		struct qpnp_adc_tm_btm_param *param,
		uint32_t *low_threshold, uint32_t *high_threshold)
{
	struct qpnp_vadc_linear_graph btm_param;
	int64_t low_output = 0, high_output = 0;
	int rc = 0, sign = 0;

	rc = qpnp_get_vadc_gain_and_offset(chip, &btm_param, CALIB_ABSOLUTE);
	if (rc < 0) {
		pr_err("Could not acquire gain and offset\n");
		return rc;
	}

	/* Convert to Kelvin and account for voltage to be written as 2mV/K */
	low_output = (param->low_temp + KELVINMIL_DEGMIL) * 2;
	/* Convert to voltage threshold */
	low_output = (low_output - QPNP_ADC_625_UV) * btm_param.dy;
	if (low_output < 0) {
		sign = 1;
		low_output = -low_output;
	}
	do_div(low_output, QPNP_ADC_625_UV);
	if (sign)
		low_output = -low_output;
	low_output += btm_param.adc_gnd;

	sign = 0;
	/* Convert to Kelvin and account for voltage to be written as 2mV/K */
	high_output = (param->high_temp + KELVINMIL_DEGMIL) * 2;
	/* Convert to voltage threshold */
	high_output = (high_output - QPNP_ADC_625_UV) * btm_param.dy;
	if (high_output < 0) {
		sign = 1;
		high_output = -high_output;
	}
	do_div(high_output, QPNP_ADC_625_UV);
	if (sign)
		high_output = -high_output;
	high_output += btm_param.adc_gnd;

	*low_threshold = (uint32_t) low_output;
	*high_threshold = (uint32_t) high_output;
	pr_debug("high_temp:%d, low_temp:%d\n", param->high_temp,
				param->low_temp);
	pr_debug("adc_code_high:%x, adc_code_low:%x\n", *high_threshold,
				*low_threshold);

	return 0;
}
EXPORT_SYMBOL(qpnp_adc_scale_millidegc_pmic_voltage_thr);

/* Scales the ADC code to degC using the mapping
 * table for the XO thermistor.
 */
int32_t qpnp_adc_tdkntcg_therm(struct qpnp_vadc_chip *chip,
		int32_t adc_code,
		const struct qpnp_adc_properties *adc_properties,
		const struct qpnp_vadc_chan_properties *chan_properties,
		struct qpnp_vadc_result *adc_chan_result)
{
	int64_t xo_thm = 0;

	if (!chan_properties || !chan_properties->offset_gain_numerator ||
		!chan_properties->offset_gain_denominator || !adc_properties
		|| !adc_chan_result)
		return -EINVAL;

	xo_thm = qpnp_adc_scale_ratiometric_calib(adc_code,
			adc_properties, chan_properties);

	qpnp_adc_map_voltage_temp(adcmap_100k_104ef_104fb,
		ARRAY_SIZE(adcmap_100k_104ef_104fb),
		xo_thm, &adc_chan_result->physical);

	return 0;
}
EXPORT_SYMBOL(qpnp_adc_tdkntcg_therm);

/* [PLATFORM]-Add-BEGIN by TCTNB.YuBin, 2014/12/24, read battery id for byd and scud temperature */
#if defined(CONFIG_TCT_8X16_ALTO4)
int64_t function_battery_id_for_qpnp_adc_common(void);
#endif
/* [PLATFORM]-Add-END by TCTNB.YuBin */

int32_t qpnp_adc_scale_batt_therm(struct qpnp_vadc_chip *chip,
		int32_t adc_code,
		const struct qpnp_adc_properties *adc_properties,
		const struct qpnp_vadc_chan_properties *chan_properties,
		struct qpnp_vadc_result *adc_chan_result)
{
	int64_t bat_voltage = 0;
/* [PLATFORM]-Add-BEGIN by TCTNB.YuBin, 2014/12/24, read battery id for byd and scud temperature */
#if defined(CONFIG_TCT_8X16_ALTO4)
	int64_t battery_id_for_qpnp_adc_common = function_battery_id_for_qpnp_adc_common();
#endif
/* [PLATFORM]-Add-END by TCTNB.YuBin */

	bat_voltage = qpnp_adc_scale_ratiometric_calib(adc_code,
			adc_properties, chan_properties);

	adc_chan_result->measurement = bat_voltage;

/* [PLATFORM]-Add-BEGIN by TCTNB.YuBin, 2014/12/24, read battery id for byd and scud temperature */
#if defined(CONFIG_TCT_8X16_ALTO4)
	if(battery_id_for_qpnp_adc_common>1080000 && battery_id_for_qpnp_adc_common <1320000)		// SCUD [1.08V, 1.32]
		return qpnp_adc_map_temp_voltage(
				adcmap_btm_threshold_scud,
				ARRAY_SIZE(adcmap_btm_threshold_scud),
				bat_voltage,
				&adc_chan_result->physical);
	else
		return qpnp_adc_map_temp_voltage(
				adcmap_btm_threshold_byd,
				ARRAY_SIZE(adcmap_btm_threshold_byd),
				bat_voltage,
				&adc_chan_result->physical);
#else
	return qpnp_adc_map_temp_voltage(
			adcmap_btm_threshold,
			ARRAY_SIZE(adcmap_btm_threshold),
			bat_voltage,
			&adc_chan_result->physical);
#endif
/* [PLATFORM]-Add-END by TCTNB.YuBin */
}
EXPORT_SYMBOL(qpnp_adc_scale_batt_therm);

int32_t qpnp_adc_scale_qrd_batt_therm(struct qpnp_vadc_chip *chip,
		int32_t adc_code,
		const struct qpnp_adc_properties *adc_properties,
		const struct qpnp_vadc_chan_properties *chan_properties,
		struct qpnp_vadc_result *adc_chan_result)
{
	int64_t bat_voltage = 0;

	bat_voltage = qpnp_adc_scale_ratiometric_calib(adc_code,
			adc_properties, chan_properties);

	adc_chan_result->measurement = bat_voltage;

	return qpnp_adc_map_temp_voltage(
			adcmap_qrd_btm_threshold,
			ARRAY_SIZE(adcmap_qrd_btm_threshold),
			bat_voltage,
			&adc_chan_result->physical);
}
EXPORT_SYMBOL(qpnp_adc_scale_qrd_batt_therm);

int32_t qpnp_adc_scale_qrd_skuaa_batt_therm(struct qpnp_vadc_chip *chip,
		int32_t adc_code,
		const struct qpnp_adc_properties *adc_properties,
		const struct qpnp_vadc_chan_properties *chan_properties,
		struct qpnp_vadc_result *adc_chan_result)
{
	int64_t bat_voltage = 0;

	bat_voltage = qpnp_adc_scale_ratiometric_calib(adc_code,
			adc_properties, chan_properties);

	adc_chan_result->measurement = bat_voltage;

	return qpnp_adc_map_temp_voltage(
			adcmap_qrd_skuaa_btm_threshold,
			ARRAY_SIZE(adcmap_qrd_skuaa_btm_threshold),
			bat_voltage,
			&adc_chan_result->physical);
}
EXPORT_SYMBOL(qpnp_adc_scale_qrd_skuaa_batt_therm);

int32_t qpnp_adc_scale_qrd_skug_batt_therm(struct qpnp_vadc_chip *chip,
		int32_t adc_code,
		const struct qpnp_adc_properties *adc_properties,
		const struct qpnp_vadc_chan_properties *chan_properties,
		struct qpnp_vadc_result *adc_chan_result)
{
	int64_t bat_voltage = 0;

	bat_voltage = qpnp_adc_scale_ratiometric_calib(adc_code,
			adc_properties, chan_properties);

	adc_chan_result->measurement = bat_voltage;

	return qpnp_adc_map_temp_voltage(
			adcmap_qrd_skug_btm_threshold,
			ARRAY_SIZE(adcmap_qrd_skug_btm_threshold),
			bat_voltage,
			&adc_chan_result->physical);
}
EXPORT_SYMBOL(qpnp_adc_scale_qrd_skug_batt_therm);

int32_t qpnp_adc_scale_qrd_skuh_batt_therm(struct qpnp_vadc_chip *chip,
		int32_t adc_code,
		const struct qpnp_adc_properties *adc_properties,
		const struct qpnp_vadc_chan_properties *chan_properties,
		struct qpnp_vadc_result *adc_chan_result)
{
	int64_t bat_voltage = 0;

	bat_voltage = qpnp_adc_scale_ratiometric_calib(adc_code,
			adc_properties, chan_properties);

	return qpnp_adc_map_temp_voltage(
			adcmap_qrd_skuh_btm_threshold,
			ARRAY_SIZE(adcmap_qrd_skuh_btm_threshold),
			bat_voltage,
			&adc_chan_result->physical);
}
EXPORT_SYMBOL(qpnp_adc_scale_qrd_skuh_batt_therm);

int32_t qpnp_adc_scale_smb_batt_therm(struct qpnp_vadc_chip *chip,
		int32_t adc_code,
		const struct qpnp_adc_properties *adc_properties,
		const struct qpnp_vadc_chan_properties *chan_properties,
		struct qpnp_vadc_result *adc_chan_result)
{
	int64_t bat_voltage = 0;

	bat_voltage = qpnp_adc_scale_ratiometric_calib(adc_code,
			adc_properties, chan_properties);

	return qpnp_adc_map_temp_voltage(
			adcmap_smb_batt_therm,
			ARRAY_SIZE(adcmap_smb_batt_therm),
			bat_voltage,
			&adc_chan_result->physical);
}
EXPORT_SYMBOL(qpnp_adc_scale_smb_batt_therm);

int32_t qpnp_adc_scale_therm_pu1(struct qpnp_vadc_chip *chip,
		int32_t adc_code,
		const struct qpnp_adc_properties *adc_properties,
		const struct qpnp_vadc_chan_properties *chan_properties,
		struct qpnp_vadc_result *adc_chan_result)
{
	int64_t therm_voltage = 0;

	therm_voltage = qpnp_adc_scale_ratiometric_calib(adc_code,
			adc_properties, chan_properties);

	qpnp_adc_map_voltage_temp(adcmap_150k_104ef_104fb,
		ARRAY_SIZE(adcmap_150k_104ef_104fb),
		therm_voltage, &adc_chan_result->physical);

	return 0;
}
EXPORT_SYMBOL(qpnp_adc_scale_therm_pu1);

int32_t qpnp_adc_scale_therm_pu2(struct qpnp_vadc_chip *chip,
		int32_t adc_code,
		const struct qpnp_adc_properties *adc_properties,
		const struct qpnp_vadc_chan_properties *chan_properties,
		struct qpnp_vadc_result *adc_chan_result)
{
	int64_t therm_voltage = 0;

	therm_voltage = qpnp_adc_scale_ratiometric_calib(adc_code,
			adc_properties, chan_properties);

	qpnp_adc_map_voltage_temp(adcmap_100k_104ef_104fb,
		ARRAY_SIZE(adcmap_100k_104ef_104fb),
		therm_voltage, &adc_chan_result->physical);

	return 0;
}
EXPORT_SYMBOL(qpnp_adc_scale_therm_pu2);

int32_t qpnp_adc_tm_scale_voltage_therm_pu2(struct qpnp_vadc_chip *chip,
					uint32_t reg, int64_t *result)
{
	int64_t adc_voltage = 0;
	struct qpnp_vadc_linear_graph param1;
	int negative_offset;

	qpnp_get_vadc_gain_and_offset(chip, &param1, CALIB_RATIOMETRIC);

	adc_voltage = (reg - param1.adc_gnd) * param1.adc_vref;
	if (adc_voltage < 0) {
		negative_offset = 1;
		adc_voltage = -adc_voltage;
	}

	do_div(adc_voltage, param1.dy);

	qpnp_adc_map_voltage_temp(adcmap_100k_104ef_104fb,
		ARRAY_SIZE(adcmap_100k_104ef_104fb),
		adc_voltage, result);
	if (negative_offset)
		adc_voltage = -adc_voltage;

	return 0;
}
EXPORT_SYMBOL(qpnp_adc_tm_scale_voltage_therm_pu2);

int32_t qpnp_adc_tm_scale_therm_voltage_pu2(struct qpnp_vadc_chip *chip,
				struct qpnp_adc_tm_config *param)
{
	struct qpnp_vadc_linear_graph param1;
	int rc;

	qpnp_get_vadc_gain_and_offset(chip, &param1, CALIB_RATIOMETRIC);

	rc = qpnp_adc_map_temp_voltage(adcmap_100k_104ef_104fb,
		ARRAY_SIZE(adcmap_100k_104ef_104fb),
		param->low_thr_temp, &param->low_thr_voltage);
	if (rc)
		return rc;

	param->low_thr_voltage *= param1.dy;
	do_div(param->low_thr_voltage, param1.adc_vref);
	param->low_thr_voltage += param1.adc_gnd;

	rc = qpnp_adc_map_temp_voltage(adcmap_100k_104ef_104fb,
		ARRAY_SIZE(adcmap_100k_104ef_104fb),
		param->high_thr_temp, &param->high_thr_voltage);
	if (rc)
		return rc;

	param->high_thr_voltage *= param1.dy;
	do_div(param->high_thr_voltage, param1.adc_vref);
	param->high_thr_voltage += param1.adc_gnd;

	return 0;
}
EXPORT_SYMBOL(qpnp_adc_tm_scale_therm_voltage_pu2);

int32_t qpnp_adc_scale_therm_ncp03(struct qpnp_vadc_chip *chip,
		int32_t adc_code,
		const struct qpnp_adc_properties *adc_properties,
		const struct qpnp_vadc_chan_properties *chan_properties,
		struct qpnp_vadc_result *adc_chan_result)
{
	int64_t therm_voltage = 0;

	therm_voltage = qpnp_adc_scale_ratiometric_calib(adc_code,
			adc_properties, chan_properties);

	qpnp_adc_map_voltage_temp(adcmap_ncp03wf683,
		ARRAY_SIZE(adcmap_ncp03wf683),
		therm_voltage, &adc_chan_result->physical);

	return 0;
}
EXPORT_SYMBOL(qpnp_adc_scale_therm_ncp03);

int32_t qpnp_adc_scale_batt_id(struct qpnp_vadc_chip *chip,
		int32_t adc_code,
		const struct qpnp_adc_properties *adc_properties,
		const struct qpnp_vadc_chan_properties *chan_properties,
		struct qpnp_vadc_result *adc_chan_result)
{
	int64_t batt_id_voltage = 0;

	batt_id_voltage = qpnp_adc_scale_ratiometric_calib(adc_code,
			adc_properties, chan_properties);
	adc_chan_result->physical = batt_id_voltage;
	adc_chan_result->physical = adc_chan_result->measurement;

	return 0;
}
EXPORT_SYMBOL(qpnp_adc_scale_batt_id);

int32_t qpnp_adc_scale_default(struct qpnp_vadc_chip *vadc,
		int32_t adc_code,
		const struct qpnp_adc_properties *adc_properties,
		const struct qpnp_vadc_chan_properties *chan_properties,
		struct qpnp_vadc_result *adc_chan_result)
{
	bool negative_rawfromoffset = 0, negative_offset = 0;
	int64_t scale_voltage = 0;

	if (!chan_properties || !chan_properties->offset_gain_numerator ||
		!chan_properties->offset_gain_denominator || !adc_properties
		|| !adc_chan_result)
		return -EINVAL;

	scale_voltage = (adc_code -
		chan_properties->adc_graph[chan_properties->calib_type].adc_gnd)
		* chan_properties->adc_graph[chan_properties->calib_type].dx;
	if (scale_voltage < 0) {
		negative_offset = 1;
		scale_voltage = -scale_voltage;
	}
	do_div(scale_voltage,
		chan_properties->adc_graph[chan_properties->calib_type].dy);
	if (negative_offset)
		scale_voltage = -scale_voltage;

	if (chan_properties->calib_type == CALIB_ABSOLUTE)
		scale_voltage +=
		chan_properties->adc_graph[chan_properties->calib_type].dx;
	else
		scale_voltage *= 1000;

	if (scale_voltage < 0) {
		if (adc_properties->bipolar) {
			scale_voltage = -scale_voltage;
			negative_rawfromoffset = 1;
		} else {
			scale_voltage = 0;
		}
	}

	adc_chan_result->measurement = scale_voltage *
				chan_properties->offset_gain_denominator;

	/* do_div only perform positive integer division! */
	do_div(adc_chan_result->measurement,
				chan_properties->offset_gain_numerator);

	if (negative_rawfromoffset)
		adc_chan_result->measurement = -adc_chan_result->measurement;

	/*
	 * Note: adc_chan_result->measurement is in the unit of
	 * adc_properties.adc_reference. For generic channel processing,
	 * channel measurement is a scale/ratio relative to the adc
	 * reference input
	 */
	adc_chan_result->physical = adc_chan_result->measurement;

	return 0;
}
EXPORT_SYMBOL(qpnp_adc_scale_default);

int32_t qpnp_adc_usb_scaler(struct qpnp_vadc_chip *chip,
		struct qpnp_adc_tm_btm_param *param,
		uint32_t *low_threshold, uint32_t *high_threshold)
{
	struct qpnp_vadc_linear_graph usb_param;

	qpnp_get_vadc_gain_and_offset(chip, &usb_param, CALIB_RATIOMETRIC);

	*low_threshold = param->low_thr * usb_param.dy;
	do_div(*low_threshold, usb_param.adc_vref);
	*low_threshold += usb_param.adc_gnd;

	*high_threshold = param->high_thr * usb_param.dy;
	do_div(*high_threshold, usb_param.adc_vref);
	*high_threshold += usb_param.adc_gnd;

	pr_debug("high_volt:%d, low_volt:%d\n", param->high_thr,
				param->low_thr);
	return 0;
}
EXPORT_SYMBOL(qpnp_adc_usb_scaler);

int32_t qpnp_adc_vbatt_rscaler(struct qpnp_vadc_chip *chip,
		struct qpnp_adc_tm_btm_param *param,
		uint32_t *low_threshold, uint32_t *high_threshold)
{
	struct qpnp_vadc_linear_graph vbatt_param;
	int rc = 0, sign = 0;
	int64_t low_thr = 0, high_thr = 0;

	rc = qpnp_get_vadc_gain_and_offset(chip, &vbatt_param, CALIB_ABSOLUTE);
	if (rc < 0)
		return rc;

	low_thr = (((param->low_thr/3) - QPNP_ADC_625_UV) *
				vbatt_param.dy);
	if (low_thr < 0) {
		sign = 1;
		low_thr = -low_thr;
	}
	do_div(low_thr, QPNP_ADC_625_UV);
	if (sign)
		low_thr = -low_thr;
	*low_threshold = low_thr + vbatt_param.adc_gnd;

	sign = 0;
	high_thr = (((param->high_thr/3) - QPNP_ADC_625_UV) *
				vbatt_param.dy);
	if (high_thr < 0) {
		sign = 1;
		high_thr = -high_thr;
	}
	do_div(high_thr, QPNP_ADC_625_UV);
	if (sign)
		high_thr = -high_thr;
	*high_threshold = high_thr + vbatt_param.adc_gnd;

	pr_debug("high_volt:%d, low_volt:%d\n", param->high_thr,
				param->low_thr);
	pr_debug("adc_code_high:%x, adc_code_low:%x\n", *high_threshold,
				*low_threshold);
	return 0;
}
EXPORT_SYMBOL(qpnp_adc_vbatt_rscaler);

int32_t qpnp_adc_absolute_rthr(struct qpnp_vadc_chip *chip,
		struct qpnp_adc_tm_btm_param *param,
		uint32_t *low_threshold, uint32_t *high_threshold)
{
	struct qpnp_vadc_linear_graph vbatt_param;
	int rc = 0, sign = 0;
	int64_t low_thr = 0, high_thr = 0;

	rc = qpnp_get_vadc_gain_and_offset(chip, &vbatt_param, CALIB_ABSOLUTE);
	if (rc < 0)
		return rc;

	low_thr = (((param->low_thr) - QPNP_ADC_625_UV) * vbatt_param.dy);
	if (low_thr < 0) {
		sign = 1;
		low_thr = -low_thr;
	}
	do_div(low_thr, QPNP_ADC_625_UV);
	if (sign)
		low_thr = -low_thr;
	*low_threshold = low_thr + vbatt_param.adc_gnd;

	sign = 0;
	high_thr = (((param->high_thr) - QPNP_ADC_625_UV) * vbatt_param.dy);
	if (high_thr < 0) {
		sign = 1;
		high_thr = -high_thr;
	}
	do_div(high_thr, QPNP_ADC_625_UV);
	if (sign)
		high_thr = -high_thr;
	*high_threshold = high_thr + vbatt_param.adc_gnd;

	pr_debug("high_volt:%d, low_volt:%d\n", param->high_thr,
				param->low_thr);
	pr_debug("adc_code_high:%x, adc_code_low:%x\n", *high_threshold,
				*low_threshold);
	return 0;
}
EXPORT_SYMBOL(qpnp_adc_absolute_rthr);

int32_t qpnp_vadc_absolute_rthr(struct qpnp_vadc_chip *chip,
		const struct qpnp_vadc_chan_properties *chan_prop,
		struct qpnp_adc_tm_btm_param *param,
		uint32_t *low_threshold, uint32_t *high_threshold)
{
	struct qpnp_vadc_linear_graph vbatt_param;
	int rc = 0, sign = 0;
	int64_t low_thr = 0, high_thr = 0;

	if (!chan_prop || !chan_prop->offset_gain_numerator ||
		!chan_prop->offset_gain_denominator)
		return -EINVAL;

	rc = qpnp_get_vadc_gain_and_offset(chip, &vbatt_param, CALIB_ABSOLUTE);
	if (rc < 0)
		return rc;

	low_thr = (((param->low_thr)/chan_prop->offset_gain_denominator
					- QPNP_ADC_625_UV) * vbatt_param.dy);
	if (low_thr < 0) {
		sign = 1;
		low_thr = -low_thr;
	}
	low_thr = low_thr * chan_prop->offset_gain_numerator;
	do_div(low_thr, QPNP_ADC_625_UV);
	if (sign)
		low_thr = -low_thr;
	*low_threshold = low_thr + vbatt_param.adc_gnd;

	sign = 0;
	high_thr = (((param->high_thr)/chan_prop->offset_gain_denominator
					- QPNP_ADC_625_UV) * vbatt_param.dy);
	if (high_thr < 0) {
		sign = 1;
		high_thr = -high_thr;
	}
	high_thr = high_thr * chan_prop->offset_gain_numerator;
	do_div(high_thr, QPNP_ADC_625_UV);
	if (sign)
		high_thr = -high_thr;
	*high_threshold = high_thr + vbatt_param.adc_gnd;

	pr_debug("high_volt:%d, low_volt:%d\n", param->high_thr,
				param->low_thr);
	pr_debug("adc_code_high:%x, adc_code_low:%x\n", *high_threshold,
				*low_threshold);
	return 0;
}
EXPORT_SYMBOL(qpnp_vadc_absolute_rthr);

int32_t qpnp_adc_btm_scaler(struct qpnp_vadc_chip *chip,
		struct qpnp_adc_tm_btm_param *param,
		uint32_t *low_threshold, uint32_t *high_threshold)
{
	struct qpnp_vadc_linear_graph btm_param;
	int64_t low_output = 0, high_output = 0;
	int rc = 0;
/* [PLATFORM]-Add-BEGIN by TCTNB.YuBin, 2014/12/24, read battery id for byd and scud temperature */
#if defined(CONFIG_TCT_8X16_ALTO4)
	int64_t battery_id_for_qpnp_adc_common = function_battery_id_for_qpnp_adc_common();
#endif
/* [PLATFORM]-Add-END by TCTNB.YuBin */

	qpnp_get_vadc_gain_and_offset(chip, &btm_param, CALIB_RATIOMETRIC);

	pr_debug("warm_temp:%d and cool_temp:%d\n", param->high_temp,
				param->low_temp);
/* [PLATFORM]-Add-BEGIN by TCTNB.YuBin, 2014/12/24, read battery id for byd and scud temperature */
#if defined(CONFIG_TCT_8X16_ALTO4)
	if(battery_id_for_qpnp_adc_common>1080000 && battery_id_for_qpnp_adc_common <1320000)		// SCUD [1.08V, 1.32]
		rc = qpnp_adc_map_voltage_temp(
			adcmap_btm_threshold_scud,
			ARRAY_SIZE(adcmap_btm_threshold_scud),
			(param->low_temp),
			&low_output);
	else
		rc = qpnp_adc_map_voltage_temp(
			adcmap_btm_threshold_byd,
			ARRAY_SIZE(adcmap_btm_threshold_byd),
			(param->low_temp),
			&low_output);
#else
	rc = qpnp_adc_map_voltage_temp(
		adcmap_btm_threshold,
		ARRAY_SIZE(adcmap_btm_threshold),
		(param->low_temp),
		&low_output);
#endif
/* [PLATFORM]-Add-END by TCTNB.YuBin */
	if (rc) {
		pr_debug("low_temp mapping failed with %d\n", rc);
		return rc;
	}

	pr_debug("low_output:%lld\n", low_output);
	low_output *= btm_param.dy;
	do_div(low_output, btm_param.adc_vref);
	low_output += btm_param.adc_gnd;
/* [PLATFORM]-Add-BEGIN by TCTNB.YuBin, 2014/12/24, read battery id for byd and scud temperature */
#if defined(CONFIG_TCT_8X16_ALTO4)
	if(battery_id_for_qpnp_adc_common>1080000 && battery_id_for_qpnp_adc_common <1320000)		// SCUD [1.08V, 1.32]
		rc = qpnp_adc_map_voltage_temp(
			adcmap_btm_threshold_scud,
			ARRAY_SIZE(adcmap_btm_threshold_scud),
			(param->high_temp),
			&high_output);
	else
		rc = qpnp_adc_map_voltage_temp(
			adcmap_btm_threshold_byd,
			ARRAY_SIZE(adcmap_btm_threshold_byd),
			(param->high_temp),
			&high_output);
#else
	rc = qpnp_adc_map_voltage_temp(
		adcmap_btm_threshold,
		ARRAY_SIZE(adcmap_btm_threshold),
		(param->high_temp),
		&high_output);
#endif
/* [PLATFORM]-Add-END by TCTNB.YuBin */
	if (rc) {
		pr_debug("high temp mapping failed with %d\n", rc);
		return rc;
	}

	pr_debug("high_output:%lld\n", high_output);
	high_output *= btm_param.dy;
	do_div(high_output, btm_param.adc_vref);
	high_output += btm_param.adc_gnd;

	/* btm low temperature correspondes to high voltage threshold */
	*low_threshold = high_output;
	/* btm high temperature correspondes to low voltage threshold */
	*high_threshold = low_output;

	pr_debug("high_volt:%d, low_volt:%d\n", *high_threshold,
				*low_threshold);
	return 0;
}
EXPORT_SYMBOL(qpnp_adc_btm_scaler);

int32_t qpnp_adc_qrd_skuh_btm_scaler(struct qpnp_vadc_chip *chip,
		struct qpnp_adc_tm_btm_param *param,
		uint32_t *low_threshold, uint32_t *high_threshold)
{
	struct qpnp_vadc_linear_graph btm_param;
	int64_t low_output = 0, high_output = 0;
	int rc = 0;

	qpnp_get_vadc_gain_and_offset(chip, &btm_param, CALIB_RATIOMETRIC);

	pr_debug("warm_temp:%d and cool_temp:%d\n", param->high_temp,
				param->low_temp);
	rc = qpnp_adc_map_voltage_temp(
		adcmap_qrd_skuh_btm_threshold,
		ARRAY_SIZE(adcmap_qrd_skuh_btm_threshold),
		(param->low_temp),
		&low_output);
	if (rc) {
		pr_debug("low_temp mapping failed with %d\n", rc);
		return rc;
	}

	pr_debug("low_output:%lld\n", low_output);
	low_output *= btm_param.dy;
	do_div(low_output, btm_param.adc_vref);
	low_output += btm_param.adc_gnd;

	rc = qpnp_adc_map_voltage_temp(
		adcmap_qrd_skuh_btm_threshold,
		ARRAY_SIZE(adcmap_qrd_skuh_btm_threshold),
		(param->high_temp),
		&high_output);
	if (rc) {
		pr_debug("high temp mapping failed with %d\n", rc);
		return rc;
	}

	pr_debug("high_output:%lld\n", high_output);
	high_output *= btm_param.dy;
	do_div(high_output, btm_param.adc_vref);
	high_output += btm_param.adc_gnd;

	/* btm low temperature correspondes to high voltage threshold */
	*low_threshold = high_output;
	/* btm high temperature correspondes to low voltage threshold */
	*high_threshold = low_output;

	pr_debug("high_volt:%d, low_volt:%d\n", *high_threshold,
				*low_threshold);
	return 0;
}
EXPORT_SYMBOL(qpnp_adc_qrd_skuh_btm_scaler);

int32_t qpnp_adc_smb_btm_rscaler(struct qpnp_vadc_chip *chip,
		struct qpnp_adc_tm_btm_param *param,
		uint32_t *low_threshold, uint32_t *high_threshold)
{
	struct qpnp_vadc_linear_graph btm_param;
	int64_t low_output = 0, high_output = 0;
	int rc = 0;

	qpnp_get_vadc_gain_and_offset(chip, &btm_param, CALIB_RATIOMETRIC);

	pr_debug("warm_temp:%d and cool_temp:%d\n", param->high_temp,
				param->low_temp);
	rc = qpnp_adc_map_voltage_temp(
		adcmap_smb_batt_therm,
		ARRAY_SIZE(adcmap_smb_batt_therm),
		(param->low_temp),
		&low_output);
	if (rc) {
		pr_debug("low_temp mapping failed with %d\n", rc);
		return rc;
	}

	pr_debug("low_output:%lld\n", low_output);
	low_output *= btm_param.dy;
	do_div(low_output, btm_param.adc_vref);
	low_output += btm_param.adc_gnd;

	rc = qpnp_adc_map_voltage_temp(
		adcmap_smb_batt_therm,
		ARRAY_SIZE(adcmap_smb_batt_therm),
		(param->high_temp),
		&high_output);
	if (rc) {
		pr_debug("high temp mapping failed with %d\n", rc);
		return rc;
	}

	pr_debug("high_output:%lld\n", high_output);
	high_output *= btm_param.dy;
	do_div(high_output, btm_param.adc_vref);
	high_output += btm_param.adc_gnd;

	/* btm low temperature correspondes to high voltage threshold */
	*low_threshold = high_output;
	/* btm high temperature correspondes to low voltage threshold */
	*high_threshold = low_output;

	pr_debug("high_volt:%d, low_volt:%d\n", *high_threshold,
				*low_threshold);
	return 0;
}
EXPORT_SYMBOL(qpnp_adc_smb_btm_rscaler);

int32_t qpnp_vadc_check_result(int32_t *data, bool recalib_check)
{
	if (recalib_check)
		return 0;

	if (*data < QPNP_VADC_MIN_ADC_CODE)
		*data = QPNP_VADC_MIN_ADC_CODE;
	else if (*data > QPNP_VADC_MAX_ADC_CODE)
		*data = QPNP_VADC_MAX_ADC_CODE;

	return 0;
}
EXPORT_SYMBOL(qpnp_vadc_check_result);

int qpnp_adc_get_revid_version(struct device *dev)
{
	struct pmic_revid_data *revid_data;
	struct device_node *revid_dev_node;

	revid_dev_node = of_parse_phandle(dev->of_node,
						"qcom,pmic-revid", 0);
	if (!revid_dev_node) {
		pr_debug("Missing qcom,pmic-revid property\n");
		return -EINVAL;
	}

	revid_data = get_revid_data(revid_dev_node);
	if (IS_ERR(revid_data)) {
		pr_debug("revid error rc = %ld\n", PTR_ERR(revid_data));
		return -EINVAL;
	}

	if ((revid_data->rev1 == PM8941_V3P1_REV1) &&
		(revid_data->rev2 == PM8941_V3P1_REV2) &&
		(revid_data->rev3 == PM8941_V3P1_REV3) &&
		(revid_data->rev4 == PM8941_V3P1_REV4) &&
		(revid_data->pmic_type == PM8941_V3P1_TYPE) &&
		(revid_data->pmic_subtype == PM8941_V3P1_SUBTYPE))
			return QPNP_REV_ID_8941_3_1;
	else if ((revid_data->rev1 == PM8941_V3P0_REV1) &&
		(revid_data->rev2 == PM8941_V3P0_REV2) &&
		(revid_data->rev3 == PM8941_V3P0_REV3) &&
		(revid_data->rev4 == PM8941_V3P0_REV4) &&
		(revid_data->pmic_type == PM8941_V3P0_TYPE) &&
		(revid_data->pmic_subtype == PM8941_V3P0_SUBTYPE))
			return QPNP_REV_ID_8941_3_0;
	else if ((revid_data->rev1 == PM8941_V2P0_REV1) &&
		(revid_data->rev2 == PM8941_V2P0_REV2) &&
		(revid_data->rev3 == PM8941_V2P0_REV3) &&
		(revid_data->rev4 == PM8941_V2P0_REV4) &&
		(revid_data->pmic_type == PM8941_V2P0_TYPE) &&
		(revid_data->pmic_subtype == PM8941_V2P0_SUBTYPE))
			return QPNP_REV_ID_8941_2_0;
	else if ((revid_data->rev1 == PM8226_V2P2_REV1) &&
		(revid_data->rev2 == PM8226_V2P2_REV2) &&
		(revid_data->rev3 == PM8226_V2P2_REV3) &&
		(revid_data->rev4 == PM8226_V2P2_REV4) &&
		(revid_data->pmic_type == PM8226_V2P2_TYPE) &&
		(revid_data->pmic_subtype == PM8226_V2P2_SUBTYPE))
			return QPNP_REV_ID_8026_2_2;
	else if ((revid_data->rev1 == PM8226_V2P1_REV1) &&
		(revid_data->rev2 == PM8226_V2P1_REV2) &&
		(revid_data->rev3 == PM8226_V2P1_REV3) &&
		(revid_data->rev4 == PM8226_V2P1_REV4) &&
		(revid_data->pmic_type == PM8226_V2P1_TYPE) &&
		(revid_data->pmic_subtype == PM8226_V2P1_SUBTYPE))
			return QPNP_REV_ID_8026_2_1;
	else if ((revid_data->rev1 == PM8226_V2P0_REV1) &&
		(revid_data->rev2 == PM8226_V2P0_REV2) &&
		(revid_data->rev3 == PM8226_V2P0_REV3) &&
		(revid_data->rev4 == PM8226_V2P0_REV4) &&
		(revid_data->pmic_type == PM8226_V2P0_TYPE) &&
		(revid_data->pmic_subtype == PM8226_V2P0_SUBTYPE))
			return QPNP_REV_ID_8026_2_0;
	else if ((revid_data->rev1 == PM8226_V1P0_REV1) &&
		(revid_data->rev2 == PM8226_V1P0_REV2) &&
		(revid_data->rev3 == PM8226_V1P0_REV3) &&
		(revid_data->rev4 == PM8226_V1P0_REV4) &&
		(revid_data->pmic_type == PM8226_V1P0_TYPE) &&
		(revid_data->pmic_subtype == PM8226_V1P0_SUBTYPE))
			return QPNP_REV_ID_8026_1_0;
	else if ((revid_data->rev1 == PM8110_V1P0_REV1) &&
		(revid_data->rev2 == PM8110_V1P0_REV2) &&
		(revid_data->rev3 == PM8110_V1P0_REV3) &&
		(revid_data->rev4 == PM8110_V1P0_REV4) &&
		(revid_data->pmic_type == PM8110_V1P0_TYPE) &&
		(revid_data->pmic_subtype == PM8110_V1P0_SUBTYPE))
			return QPNP_REV_ID_8110_1_0;
	else if ((revid_data->rev1 == PM8110_V2P0_REV1) &&
		(revid_data->rev2 == PM8110_V2P0_REV2) &&
		(revid_data->rev3 == PM8110_V2P0_REV3) &&
		(revid_data->rev4 == PM8110_V2P0_REV4) &&
		(revid_data->pmic_type == PM8110_V2P0_TYPE) &&
		(revid_data->pmic_subtype == PM8110_V2P0_SUBTYPE))
			return QPNP_REV_ID_8110_2_0;
	else if ((revid_data->rev1 == PM8916_V1P0_REV1) &&
		(revid_data->rev2 == PM8916_V1P0_REV2) &&
		(revid_data->rev3 == PM8916_V1P0_REV3) &&
		(revid_data->rev4 == PM8916_V1P0_REV4) &&
		(revid_data->pmic_type == PM8916_V1P0_TYPE) &&
		(revid_data->pmic_subtype == PM8916_V1P0_SUBTYPE))
			return QPNP_REV_ID_8916_1_0;
	else if ((revid_data->rev1 == PM8916_V1P1_REV1) &&
		(revid_data->rev2 == PM8916_V1P1_REV2) &&
		(revid_data->rev3 == PM8916_V1P1_REV3) &&
		(revid_data->rev4 == PM8916_V1P1_REV4) &&
		(revid_data->pmic_type == PM8916_V1P1_TYPE) &&
		(revid_data->pmic_subtype == PM8916_V1P1_SUBTYPE))
			return QPNP_REV_ID_8916_1_1;
	else if ((revid_data->rev1 == PM8916_V2P0_REV1) &&
		(revid_data->rev2 == PM8916_V2P0_REV2) &&
		(revid_data->rev3 == PM8916_V2P0_REV3) &&
		(revid_data->rev4 == PM8916_V2P0_REV4) &&
		(revid_data->pmic_type == PM8916_V2P0_TYPE) &&
		(revid_data->pmic_subtype == PM8916_V2P0_SUBTYPE))
			return QPNP_REV_ID_8916_2_0;
	else
		return -EINVAL;
}
EXPORT_SYMBOL(qpnp_adc_get_revid_version);

int32_t qpnp_adc_get_devicetree_data(struct spmi_device *spmi,
			struct qpnp_adc_drv *adc_qpnp)
{
	struct device_node *node = spmi->dev.of_node;
	struct resource *res;
	struct device_node *child;
	struct qpnp_adc_amux *adc_channel_list;
	struct qpnp_adc_properties *adc_prop;
	struct qpnp_adc_amux_properties *amux_prop;
	int count_adc_channel_list = 0, decimation, rc = 0, i = 0;

	if (!node)
		return -EINVAL;

	for_each_child_of_node(node, child)
		count_adc_channel_list++;

	if (!count_adc_channel_list) {
		pr_err("No channel listing\n");
		return -EINVAL;
	}

	adc_qpnp->spmi = spmi;

	adc_prop = devm_kzalloc(&spmi->dev, sizeof(struct qpnp_adc_properties),
					GFP_KERNEL);
	if (!adc_prop) {
		dev_err(&spmi->dev, "Unable to allocate memory\n");
		return -ENOMEM;
	}
	adc_channel_list = devm_kzalloc(&spmi->dev,
		((sizeof(struct qpnp_adc_amux)) * count_adc_channel_list),
				GFP_KERNEL);
	if (!adc_channel_list) {
		dev_err(&spmi->dev, "Unable to allocate memory\n");
		return -ENOMEM;
	}

	amux_prop = devm_kzalloc(&spmi->dev,
		sizeof(struct qpnp_adc_amux_properties) +
		sizeof(struct qpnp_vadc_chan_properties), GFP_KERNEL);
	if (!amux_prop) {
		dev_err(&spmi->dev, "Unable to allocate memory\n");
		return -ENOMEM;
	}

	adc_qpnp->adc_channels = adc_channel_list;
	adc_qpnp->amux_prop = amux_prop;

	for_each_child_of_node(node, child) {
		int channel_num, scaling, post_scaling, hw_settle_time;
		int fast_avg_setup, calib_type = 0, rc;
		const char *calibration_param, *channel_name;

		channel_name = of_get_property(child,
				"label", NULL) ? : child->name;
		if (!channel_name) {
			pr_err("Invalid channel name\n");
			return -EINVAL;
		}

		rc = of_property_read_u32(child, "reg", &channel_num);
		if (rc) {
			pr_err("Invalid channel num\n");
			return -EINVAL;
		}
		rc = of_property_read_u32(child, "qcom,decimation",
								&decimation);
		if (rc) {
			pr_err("Invalid channel decimation property\n");
			return -EINVAL;
		}
		if (!of_device_is_compatible(node, "qcom,qpnp-iadc")) {
			rc = of_property_read_u32(child,
				"qcom,hw-settle-time", &hw_settle_time);
			if (rc) {
				pr_err("Invalid channel hw settle time property\n");
				return -EINVAL;
			}
			rc = of_property_read_u32(child,
				"qcom,pre-div-channel-scaling", &scaling);
			if (rc) {
				pr_err("Invalid channel scaling property\n");
				return -EINVAL;
			}
			rc = of_property_read_u32(child,
				"qcom,scale-function", &post_scaling);
			if (rc) {
				pr_err("Invalid channel post scaling property\n");
				return -EINVAL;
			}
			rc = of_property_read_string(child,
				"qcom,calibration-type", &calibration_param);
			if (rc) {
				pr_err("Invalid calibration type\n");
				return -EINVAL;
			}
			if (!strcmp(calibration_param, "absolute"))
				calib_type = CALIB_ABSOLUTE;
			else if (!strcmp(calibration_param, "ratiometric"))
				calib_type = CALIB_RATIOMETRIC;
			else {
				pr_err("%s: Invalid calibration property\n",
						__func__);
				return -EINVAL;
			}
		}
		rc = of_property_read_u32(child,
				"qcom,fast-avg-setup", &fast_avg_setup);
		if (rc) {
			pr_err("Invalid channel fast average setup\n");
			return -EINVAL;
		}
		/* Individual channel properties */
		adc_channel_list[i].name = (char *)channel_name;
		adc_channel_list[i].channel_num = channel_num;
		adc_channel_list[i].adc_decimation = decimation;
		adc_channel_list[i].fast_avg_setup = fast_avg_setup;
		if (!of_device_is_compatible(node, "qcom,qpnp-iadc")) {
			adc_channel_list[i].chan_path_prescaling = scaling;
			adc_channel_list[i].adc_scale_fn = post_scaling;
			adc_channel_list[i].hw_settle_time = hw_settle_time;
			adc_channel_list[i].calib_type = calib_type;
		}
		i++;
	}

	/* Get the ADC VDD reference voltage and ADC bit resolution */
	rc = of_property_read_u32(node, "qcom,adc-vdd-reference",
			&adc_prop->adc_vdd_reference);
	if (rc) {
		pr_err("Invalid adc vdd reference property\n");
		return -EINVAL;
	}
	rc = of_property_read_u32(node, "qcom,adc-bit-resolution",
			&adc_prop->bitresolution);
	if (rc) {
		pr_err("Invalid adc bit resolution property\n");
		return -EINVAL;
	}
	adc_qpnp->adc_prop = adc_prop;

	/* Get the peripheral address */
	res = spmi_get_resource(spmi, 0, IORESOURCE_MEM, 0);
	if (!res) {
		pr_err("No base address definition\n");
		return -EINVAL;
	}

	adc_qpnp->slave = spmi->sid;
	adc_qpnp->offset = res->start;

	/* Register the ADC peripheral interrupt */
	adc_qpnp->adc_irq_eoc = spmi_get_irq_byname(spmi, NULL,
						"eoc-int-en-set");
	if (adc_qpnp->adc_irq_eoc < 0) {
		pr_err("Invalid irq\n");
		return -ENXIO;
	}

	init_completion(&adc_qpnp->adc_rslt_completion);

	return 0;
}
EXPORT_SYMBOL(qpnp_adc_get_devicetree_data);
