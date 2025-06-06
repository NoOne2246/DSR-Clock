
typedef struct {
    char *iana_timezone;
    char *posix_timezone;
} TimezoneMapping;

TimezoneMapping Mapping[] = {{"Africa/Abidjan","GMT0"},
{"Africa/Algiers","CET-1"},
{"Africa/Bissau","GMT0"},
{"Africa/Cairo","EEST"},
{"Africa/Casablanca","WET0"},
{"Africa/Ceuta","CET-1CEST,M3.5.0,M10.5.0/3"},
{"Africa/El_Aaiun","WET0"},
{"Africa/Johannesburg","SAST-2"},
{"Africa/Juba","CAT-2"},
{"Africa/Khartoum","EAT-3"},
{"Africa/Lagos","WAT-1"},
{"Africa/Maputo","CAT-2"},
{"Africa/Monrovia","GMT0"},
{"Africa/Nairobi","EAT-3"},
{"Africa/Ndjamena","WAT-1"},
{"Africa/Sao_Tome","GMT0"},
{"Africa/Tripoli","EET-2"},
{"Africa/Tunis","CET-1CEST,M3.5.0,M10.5.0/3"},
{"Africa/Windhoek","WAT-1WAST,M9.1.0,M4.1.0"},
{"America/Adak","HAST10HADT,M3.2.0,M11.1.0"},
{"America/Anchorage","AKST9AKDT,M3.2.0,M11.1.0"},
{"America/Araguaina","BRT3"},
{"America/Argentina/Buenos_Aires","ART3ARST,M10.1.0/0,M3.3.0/0"},
{"America/Argentina/Catamarca","ART3ARST,M10.1.0/0,M3.3.0/0"},
{"America/Argentina/Cordoba","ART3ARST,M10.1.0/0,M3.3.0/0"},
{"America/Argentina/Jujuy","ART3ARST,M10.1.0/0,M3.3.0/0"},
{"America/Argentina/La_Rioja","ART3ARST,M10.1.0/0,M3.3.0/0"},
{"America/Argentina/Mendoza","ART3ARST,M10.1.0/0,M3.3.0/0"},
{"America/Argentina/Rio_Gallegos","ART3ARST,M10.1.0/0,M3.3.0/0"},
{"America/Argentina/Salta","ART3ARST,M10.1.0/0,M3.3.0/0"},
{"America/Argentina/San_Juan","ART3ARST,M10.1.0/0,M3.3.0/0"},
{"America/Argentina/San_Luis","ART3"},
{"America/Argentina/Tucuman","ART3ARST,M10.1.0/0,M3.3.0/0"},
{"America/Argentina/Ushuaia","ART3ARST,M10.1.0/0,M3.3.0/0"},
{"America/Asuncion","PYT4PYST,M10.3.0/0,M3.2.0/0"},
{"America/Bahia","BRT3"},
{"America/Bahia_Banderas","CST6CDT,M4.1.0,M10.5.0"},
{"America/Barbados","AST4"},
{"America/Belem","BRT3"},
{"America/Belize","CST6"},
{"America/Boa_Vista","AMT4"},
{"America/Bogota","COT5"},
{"America/Boise","MST7MDT,M3.2.0,M11.1.0"},
{"America/Cambridge_Bay","MST7MDT,M3.2.0,M11.1.0"},
{"America/Campo_Grande","AMT4AMST,M10.2.0/0,M2.3.0/0"},
{"America/Cancun","CST6CDT,M4.1.0,M10.5.0"},
{"America/Caracas","VET4:30"},
{"America/Cayenne","GFT3"},
{"America/Chicago","CST6CDT,M3.2.0,M11.1.0"},
{"America/Chihuahua","MST7MDT,M4.1.0,M10.5.0"},
{"America/Ciudad_Juarez","MST7MDT,M3.2.0,M11.1.0"},
{"America/Costa_Rica","CST6"},
{"America/Coyhaique","GMT+3"},
{"America/Cuiaba","AMT4AMST,M10.2.0/0,M2.3.0/0"},
{"America/Danmarkshavn","GMT0"},
{"America/Dawson","PST8PDT,M3.2.0,M11.1.0"},
{"America/Dawson_Creek","MST7"},
{"America/Denver","MST7MDT,M3.2.0,M11.1.0"},
{"America/Detroit","EST5EDT,M3.2.0,M11.1.0"},
{"America/Edmonton","MST7MDT,M3.2.0,M11.1.0"},
{"America/Eirunepe","ACT5"},
{"America/El_Salvador","CST6"},
{"America/Fort_Nelson","MST7"},
{"America/Fortaleza","BRT3"},
{"America/Glace_Bay","AST4ADT,M3.2.0,M11.1.0"},
{"America/Goose_Bay","AST4ADT,M3.2.0/0:01,M11.1.0/0:01"},
{"America/Grand_Turk","EST5EDT,M3.2.0,M11.1.0"},
{"America/Guatemala","CST6"},
{"America/Guayaquil","ECT5"},
{"America/Guyana","GYT4"},
{"America/Halifax","AST4ADT,M3.2.0,M11.1.0"},
{"America/Havana","CST5CDT,M3.3.0/0,M10.5.0/1"},
{"America/Hermosillo","MST7"},
{"America/Indiana/Indianapolis","EST5EDT,M3.2.0,M11.1.0"},
{"America/Indiana/Knox","CST6CDT,M3.2.0,M11.1.0"},
{"America/Indiana/Marengo","EST5EDT,M3.2.0,M11.1.0"},
{"America/Indiana/Petersburg","EST5EDT,M3.2.0,M11.1.0"},
{"America/Indiana/Tell_City","CST6CDT,M3.2.0,M11.1.0"},
{"America/Indiana/Vevay","EST5EDT,M3.2.0,M11.1.0"},
{"America/Indiana/Vincennes","EST5EDT,M3.2.0,M11.1.0"},
{"America/Indiana/Winamac","EST5EDT,M3.2.0,M11.1.0"},
{"America/Inuvik","MST7MDT,M3.2.0,M11.1.0"},
{"America/Iqaluit","EST5EDT,M3.2.0,M11.1.0"},
{"America/Jamaica","EST5"},
{"America/Juneau","AKST9AKDT,M3.2.0,M11.1.0"},
{"America/Kentucky/Louisville","EST5EDT,M3.2.0,M11.1.0"},
{"America/Kentucky/Monticello","EST5EDT,M3.2.0,M11.1.0"},
{"America/La_Paz","BOT4"},
{"America/Lima","PET5"},
{"America/Los_Angeles","PST8PDT,M3.2.0,M11.1.0"},
{"America/Maceio","BRT3"},
{"America/Managua","CST6"},
{"America/Manaus","AMT4"},
{"America/Martinique","AST4"},
{"America/Matamoros","CST6CDT,M3.2.0,M11.1.0"},
{"America/Mazatlan","MST7MDT,M4.1.0,M10.5.0"},
{"America/Menominee","CST6CDT,M3.2.0,M11.1.0"},
{"America/Merida","CST6CDT,M4.1.0,M10.5.0"},
{"America/Metlakatla","AKST9AKDT,M3.2.0,M11.1.0"},
{"America/Mexico_City","CST6CDT,M4.1.0,M10.5.0"},
{"America/Miquelon","PMST3PMDT,M3.2.0,M11.1.0"},
{"America/Moncton","AST4ADT,M3.2.0,M11.1.0"},
{"America/Monterrey","CST6CDT,M4.1.0,M10.5.0"},
{"America/Montevideo","UYT3UYST,M10.1.0,M3.2.0"},
{"America/New_York","EST5EDT,M3.2.0,M11.1.0"},
{"America/Nome","AKST9AKDT,M3.2.0,M11.1.0"},
{"America/Noronha","FNT2"},
{"America/North_Dakota/Beulah","CST6CDT,M3.2.0,M11.1.0"},
{"America/North_Dakota/Center","CST6CDT,M3.2.0,M11.1.0"},
{"America/North_Dakota/New_Salem","CST6CDT,M3.2.0,M11.1.0"},
{"America/Nuuk",""},
{"America/Ojinaga","CST6CDT,M3.2.0,M11.1.0"},
{"America/Panama","EST5"},
{"America/Paramaribo","SRT3"},
{"America/Phoenix","MST7"},
{"America/Port-au-Prince","EST5"},
{"America/Porto_Velho","AMT4"},
{"America/Puerto_Rico","AST4"},
{"America/Punta_Arenas","GMT+3"},
{"America/Rankin_Inlet","CST6CDT,M3.2.0,M11.1.0"},
{"America/Recife","BRT3"},
{"America/Regina","CST6"},
{"America/Resolute","EST5"},
{"America/Rio_Branco","ACT5"},
{"America/Santarem","GMT+3"},
{"America/Santiago","CLST"},
{"America/Santo_Domingo","AST4"},
{"America/Sao_Paulo","BRT3BRST,M10.2.0/0,M2.3.0/0"},
{"America/Scoresbysund","EGT1EGST,M3.5.0/0,M10.5.0/1"},
{"America/Sitka","AKST9AKDT,M3.2.0,M11.1.0"},
{"America/St_Johns","NST3:30NDT,M3.2.0/0:01,M11.1.0/0:01"},
{"America/Swift_Current","CST6"},
{"America/Tegucigalpa","CST6"},
{"America/Thule","AST4ADT,M3.2.0,M11.1.0"},
{"America/Tijuana","PST8PDT,M4.1.0,M10.5.0"},
{"America/Toronto","EST5EDT,M3.2.0,M11.1.0"},
{"America/Vancouver","PST8PDT,M3.2.0,M11.1.0"},
{"America/Whitehorse","PST8PDT,M3.2.0,M11.1.0"},
{"America/Winnipeg","CST6CDT,M3.2.0,M11.1.0"},
{"America/Yakutat","AKST9AKDT,M3.2.0,M11.1.0"},
{"Antarctica/Casey","WST-8"},
{"Antarctica/Davis","DAVT-7"},
{"Antarctica/Macquarie","GMT-11"},
{"Antarctica/Mawson","MAWT-6"},
{"Antarctica/Palmer","CLST"},
{"Antarctica/Rothera","ROTT3"},
{"Antarctica/Troll","GMT0"},
{"Antarctica/Vostok","VOST-6"},
{"Asia/Almaty","ALMT-6"},
{"Asia/Amman","EET-2EEST,M3.5.4/0,M10.5.5/1"},
{"Asia/Anadyr","ANAT-12ANAST,M3.5.0,M10.5.0/3"},
{"Asia/Aqtau","AQTT-5"},
{"Asia/Aqtobe","AQTT-5"},
{"Asia/Ashgabat","TMT-5"},
{"Asia/Atyrau","GMT-5"},
{"Asia/Baghdad","AST-3"},
{"Asia/Baku","AZT-4AZST,M3.5.0/4,M10.5.0/5"},
{"Asia/Bangkok","ICT-7"},
{"Asia/Barnaul","GMT-7"},
{"Asia/Beirut","EET-2EEST,M3.5.0/0,M10.5.0/0"},
{"Asia/Bishkek","KGT-6"},
{"Asia/Chita","GMT-9"},
{"Asia/Colombo","IST-5:30"},
{"Asia/Damascus","EET-2EEST,M4.1.5/0,J274/0"},
{"Asia/Dhaka","BDT-6"},
{"Asia/Dili","TLT-9"},
{"Asia/Dubai","GST-4"},
{"Asia/Dushanbe","TJT-5"},
{"Asia/Famagusta","EET-2EEST,M3.5.0/3,M10.5.0/4"},
{"Asia/Gaza","EET-2EEST,J91/0,M9.2.4"},
{"Asia/Hebron","EET-2EEST,M3.5.5/0,M10.5.6/1"},
{"Asia/Ho_Chi_Minh","ICT-7"},
{"Asia/Hong_Kong","HKT-8"},
{"Asia/Hovd","HOVT-7"},
{"Asia/Irkutsk","IRKT-8IRKST,M3.5.0,M10.5.0/3"},
{"Asia/Jakarta","WIT-7"},
{"Asia/Jayapura","EIT-9"},
{"Asia/Jerusalem","IDDT"},
{"Asia/Kabul","AFT-4:30"},
{"Asia/Kamchatka","PETT-12PETST,M3.5.0,M10.5.0/3"},
{"Asia/Karachi","PKT-5"},
{"Asia/Kathmandu","GMT-5"},
{"Asia/Khandyga","GMT-9"},
{"Asia/Kolkata","IST-5:30"},
{"Asia/Krasnoyarsk","KRAT-7KRAST,M3.5.0,M10.5.0/3"},
{"Asia/Kuching","MYT-8"},
{"Asia/Macau","CST-8"},
{"Asia/Magadan","MAGT-11MAGST,M3.5.0,M10.5.0/3"},
{"Asia/Makassar","CIT-8"},
{"Asia/Manila","PHT-8"},
{"Asia/Nicosia","EET-2EEST,M3.5.0/3,M10.5.0/4"},
{"Asia/Novokuznetsk","GMT-7"},
{"Asia/Novosibirsk","NOVT-6NOVST,M3.5.0,M10.5.0/3"},
{"Asia/Omsk","OMST-6OMSST,M3.5.0,M10.5.0/3"},
{"Asia/Oral","ORAT-5"},
{"Asia/Pontianak","WIT-7"},
{"Asia/Pyongyang","KST-9"},
{"Asia/Qatar","AST-3"},
{"Asia/Qostanay","GMT-6"},
{"Asia/Qyzylorda","QYZT-6"},
{"Asia/Riyadh","AST-3"},
{"Asia/Sakhalin","SAKT-10SAKST,M3.5.0,M10.5.0/3"},
{"Asia/Samarkand","UZT-5"},
{"Asia/Seoul","KST-9"},
{"Asia/Shanghai","CST-8"},
{"Asia/Singapore","SGT-8"},
{"Asia/Srednekolymsk","MAGT-11"},
{"Asia/Taipei","CST-8"},
{"Asia/Tashkent","UZT-5"},
{"Asia/Tbilisi","GET-4"},
{"Asia/Tehran","IRDT"},
{"Asia/Thimphu","BTT-6"},
{"Asia/Tokyo","JST-9"},
{"Asia/Tomsk","GMT-7"},
{"Asia/Ulaanbaatar","ULAT-8"},
{"Asia/Urumqi","CST-8"},
{"Asia/Ust-Nera","GMT-10"},
{"Asia/Vladivostok","VLAT-10VLAST,M3.5.0,M10.5.0/3"},
{"Asia/Yakutsk","YAKT-9YAKST,M3.5.0,M10.5.0/3"},
{"Asia/Yangon","GMT-6:30"},
{"Asia/Yekaterinburg","YEKT-5YEKST,M3.5.0,M10.5.0/3"},
{"Asia/Yerevan","AMT-4AMST,M3.5.0,M10.5.0/3"},
{"Atlantic/Azores","AZOT1AZOST,M3.5.0/0,M10.5.0/1"},
{"Atlantic/Bermuda","AST4ADT,M3.2.0,M11.1.0"},
{"Atlantic/Canary","WET0WEST,M3.5.0/1,M10.5.0"},
{"Atlantic/Cape_Verde","CVT1"},
{"Atlantic/Faroe","WET0WEST,M3.5.0/1,M10.5.0"},
{"Atlantic/Madeira","WET0WEST,M3.5.0/1,M10.5.0"},
{"Atlantic/South_Georgia","GST2"},
{"Atlantic/Stanley","FKT4FKST,M9.1.0,M4.3.0"},
{"Australia/Adelaide","CST-9:30CST,M10.1.0,M4.1.0/3"},
{"Australia/Brisbane","EST-10"},
{"Australia/Broken_Hill","CST-9:30CST,M10.1.0,M4.1.0/3"},
{"Australia/Darwin","CST-9:30"},
{"Australia/Eucla","CWST-8:45"},
{"Australia/Hobart","AEST-10AEDT,M10.1.0,M4.1.0/3"},
{"Australia/Lindeman","EST-10"},
{"Australia/Lord_Howe","LHST-10:30LHDT-11,M10.1.0/2:30:00,M4.1.0/2:30:00"},
{"Australia/Melbourne","AEST-10AEDT,M10.1.0,M4.1.0/3"},
{"Australia/Perth","WST-8"},
{"Australia/Sydney","AEST-10AEDT,M10.1.0,M4.1.0/3"},
{"Etc/GMT","GMT0"},
{"Etc/GMT+1","GMT+1"},
{"Etc/GMT+10","GMT+10"},
{"Etc/GMT+11","GMT+11"},
{"Etc/GMT+12","GMT+12"},
{"Etc/GMT+2","GMT+2"},
{"Etc/GMT+3","GMT+3"},
{"Etc/GMT+4","GMT+4"},
{"Etc/GMT+5","GMT+5"},
{"Etc/GMT+6","GMT+6"},
{"Etc/GMT+7","GMT+7"},
{"Etc/GMT+8","GMT+8"},
{"Etc/GMT+9","GMT+9"},
{"Etc/GMT-1","GMT-1"},
{"Etc/GMT-10","GMT-10"},
{"Etc/GMT-11","GMT-11"},
{"Etc/GMT-12","GMT-12"},
{"Etc/GMT-13","GMT-13"},
{"Etc/GMT-14","GMT-14"},
{"Etc/GMT-2","GMT-2"},
{"Etc/GMT-3","GMT-3"},
{"Etc/GMT-4","GMT-4"},
{"Etc/GMT-5","GMT-5"},
{"Etc/GMT-6","GMT-6"},
{"Etc/GMT-7","GMT-7"},
{"Etc/GMT-8","GMT-8"},
{"Etc/GMT-9","GMT-9"},
{"Etc/UTC","UTC0"},
{"Europe/Andorra","CET-1CEST,M3.5.0,M10.5.0/3"},
{"Europe/Astrakhan","GMT-4"},
{"Europe/Athens","EET-2EEST,M3.5.0/3,M10.5.0/4"},
{"Europe/Belgrade","CET-1CEST,M3.5.0,M10.5.0/3"},
{"Europe/Berlin","CET-1CEST,M3.5.0,M10.5.0/3"},
{"Europe/Brussels","CET-1CEST,M3.5.0,M10.5.0/3"},
{"Europe/Bucharest","EET-2EEST,M3.5.0/3,M10.5.0/4"},
{"Europe/Budapest","CET-1CEST,M3.5.0,M10.5.0/3"},
{"Europe/Chisinau","EET-2EEST,M3.5.0/3,M10.5.0/4"},
{"Europe/Dublin","GMT0IST,M3.5.0/1,M10.5.0"},
{"Europe/Gibraltar","CET-1CEST,M3.5.0,M10.5.0/3"},
{"Europe/Helsinki","EET-2EEST,M3.5.0/3,M10.5.0/4"},
{"Europe/Istanbul","EET-2EEST,M3.5.0/3,M10.5.0/4"},
{"Europe/Kaliningrad","EET-2EEST,M3.5.0,M10.5.0/3"},
{"Europe/Kirov","MSK-3MSD,M3.5.0,M10.5.0/3"},
{"Europe/Kyiv","EET-2EEST,M3.5.0/3,M10.5.0/4"},
{"Europe/Lisbon","WET0WEST,M3.5.0/1,M10.5.0"},
{"Europe/London","GMT0BST,M3.5.0/1,M10.5.0"},
{"Europe/Madrid","CET-1CEST,M3.5.0,M10.5.0/3"},
{"Europe/Malta","CET-1CEST,M3.5.0,M10.5.0/3"},
{"Europe/Minsk","EET-2EEST,M3.5.0,M10.5.0/3"},
{"Europe/Moscow","MSK-3MSD,M3.5.0,M10.5.0/3"},
{"Europe/Paris","CET-1CEST,M3.5.0,M10.5.0/3"},
{"Europe/Prague","CET-1CEST,M3.5.0,M10.5.0/3"},
{"Europe/Riga","EET-2EEST,M3.5.0/3,M10.5.0/4"},
{"Europe/Rome","CET-1CEST,M3.5.0,M10.5.0/3"},
{"Europe/Samara","SAMT-4SAMST,M3.5.0,M10.5.0/3"},
{"Europe/Saratov","GMT-4"},
{"Europe/Simferopol","EET-2EEST,M3.5.0/3,M10.5.0/4"},
{"Europe/Sofia","EET-2EEST,M3.5.0/3,M10.5.0/4"},
{"Europe/Tallinn","EET-2EEST,M3.5.0/3,M10.5.0/4"},
{"Europe/Tirane","CET-1CEST,M3.5.0,M10.5.0/3"},
{"Europe/Ulyanovsk","SAMT-4SAMST,M3.5.0,M10.5.0/3"},
{"Europe/Vienna","CET-1CEST,M3.5.0,M10.5.0/3"},
{"Europe/Vilnius","EET-2EEST,M3.5.0/3,M10.5.0/4"},
{"Europe/Volgograd","VOLT-3VOLST,M3.5.0,M10.5.0/3"},
{"Europe/Warsaw","CET-1CEST,M3.5.0,M10.5.0/3"},
{"Europe/Zurich","CET-1CEST,M3.5.0,M10.5.0/3"},
{"Factory","GMT0"},
{"Indian/Chagos","IOT-6"},
{"Indian/Maldives","MVT-5"},
{"Indian/Mauritius","MUT-4"},
{"Pacific/Apia","WST11"},
{"Pacific/Auckland","NZST-12NZDT,M9.5.0,M4.1.0/3"},
{"Pacific/Bougainville","GMT-11"},
{"Pacific/Chatham","CHAST-12:45CHADT,M9.5.0/2:45,M4.1.0/3:45"},
{"Pacific/Easter","EASST"},
{"Pacific/Efate","VUT-11"},
{"Pacific/Fakaofo","TKT10"},
{"Pacific/Fiji","FJT-12"},
{"Pacific/Galapagos","GALT6"},
{"Pacific/Gambier","GAMT9"},
{"Pacific/Guadalcanal","SBT-11"},
{"Pacific/Guam","ChST-10"},
{"Pacific/Honolulu","HST10"},
{"Pacific/Kanton","GMT-13"},
{"Pacific/Kiritimati","LINT-14"},
{"Pacific/Kosrae","KOST-11"},
{"Pacific/Kwajalein","MHT-12"},
{"Pacific/Marquesas","MART9:30"},
{"Pacific/Nauru","NRT-12"},
{"Pacific/Niue","NUT11"},
{"Pacific/Norfolk","NFT-11:30"},
{"Pacific/Noumea","NCT-11"},
{"Pacific/Pago_Pago","SST11"},
{"Pacific/Palau","PWT-9"},
{"Pacific/Pitcairn","PST8"},
{"Pacific/Port_Moresby","PGT-10"},
{"Pacific/Rarotonga","CKT10"},
{"Pacific/Tahiti","TAHT10"},
{"Pacific/Tarawa","GILT-12"},
{"Pacific/Tongatapu","TOT-13"}};
