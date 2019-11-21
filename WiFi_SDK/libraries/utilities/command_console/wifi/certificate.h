/*
 * Copyright 2018, Cypress Semiconductor Corporation or a subsidiary of 
 * Cypress Semiconductor Corporation. All Rights Reserved.
 * 
 * This software, associated documentation and materials ("Software"),
 * is owned by Cypress Semiconductor Corporation
 * or one of its subsidiaries ("Cypress") and is protected by and subject to
 * worldwide patent protection (United States and foreign),
 * United States copyright laws and international treaty provisions.
 * Therefore, you may use this Software only as provided in the license
 * agreement accompanying the software package from which you
 * obtained this Software ("EULA").
 * If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
 * non-transferable license to copy, modify, and compile the Software
 * source code solely for use in connection with Cypress's
 * integrated circuit products. Any reproduction, modification, translation,
 * compilation, or representation of this Software except as specified
 * above is prohibited without the express written permission of Cypress.
 *
 * Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
 * reserves the right to make changes to the Software without notice. Cypress
 * does not assume any liability arising out of the application or use of the
 * Software or any product or circuit described in the Software. Cypress does
 * not authorize its products for use in any products where a malfunction or
 * failure of the Cypress product may reasonably be expected to result in
 * significant property damage, injury or death ("High Risk Product"). By
 * including Cypress's product in a High Risk Product, the manufacturer
 * of such system or application assumes all risk of such use and in doing
 * so agrees to indemnify Cypress against all liability.
 */
#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#define WIFI_ROOT_CERTIFICATE_STRING  \
"-----BEGIN CERTIFICATE-----\r\n"\
"MIIE3jCCA8agAwIBAgIJAO+iGqxU6iq2MA0GCSqGSIb3DQEBBQUAMIGSMQswCQYD\r\n"\
"VQQGEwJJTjEOMAwGA1UECBMFSW5kaWExEjAQBgNVBAcTCVNvbWV3aGVyZTEVMBMG\r\n"\
"A1UEChMMRXhhbXBsZSBJbmMuMSAwHgYJKoZIhvcNAQkBFhFhZG1pbkBleGFtcGxl\r\n"\
"LmNvbTEmMCQGA1UEAxMdRXhhbXBsZSBDZXJ0aWZpY2F0ZSBBdXRob3JpdHkwHhcN\r\n"\
"MTcxMDEwMTMyNTE1WhcNMTgxMDEwMTMyNTE1WjCBkjELMAkGA1UEBhMCSU4xDjAM\r\n"\
"BgNVBAgTBUluZGlhMRIwEAYDVQQHEwlTb21ld2hlcmUxFTATBgNVBAoTDEV4YW1w\r\n"\
"bGUgSW5jLjEgMB4GCSqGSIb3DQEJARYRYWRtaW5AZXhhbXBsZS5jb20xJjAkBgNV\r\n"\
"BAMTHUV4YW1wbGUgQ2VydGlmaWNhdGUgQXV0aG9yaXR5MIIBIjANBgkqhkiG9w0B\r\n"\
"AQEFAAOCAQ8AMIIBCgKCAQEA1fiypUBWAmMt0ToSOM90zOnmhSRZwEpymduXO8EN\r\n"\
"f7GlXGrgNCFFKJpTPkOrOJJEvAUva7D/xgJs1d8+EaUKoCGwnuPLQ7+rnHgwocip\r\n"\
"db/v9PAS8znUJrfteXwynBN6KhjyprhqUATZ2ZxPzB73T39fbHUSMkTVi3skclbt\r\n"\
"8y9xfdAgubaHzCVU+SfyMiaqOES7a/d6GFzxeYzs4YQJExhk0z7QX/5wK3/cCC7b\r\n"\
"zqeIwQpY3NEquV/1UTpPXLkDIzc/xN3B+NMnAem2jBpECXFSmqR0nW0KkPHa7DX+\r\n"\
"P/xgYQHe1i1yiq0uTV9oft7DReMKn+g1etaRgiyO05ViBQIDAQABo4IBMzCCAS8w\r\n"\
"HQYDVR0OBBYEFHZxpJGGUYkFX1GLl2b63oSwf0YlMIHHBgNVHSMEgb8wgbyAFHZx\r\n"\
"pJGGUYkFX1GLl2b63oSwf0YloYGYpIGVMIGSMQswCQYDVQQGEwJJTjEOMAwGA1UE\r\n"\
"CBMFSW5kaWExEjAQBgNVBAcTCVNvbWV3aGVyZTEVMBMGA1UEChMMRXhhbXBsZSBJ\r\n"\
"bmMuMSAwHgYJKoZIhvcNAQkBFhFhZG1pbkBleGFtcGxlLmNvbTEmMCQGA1UEAxMd\r\n"\
"RXhhbXBsZSBDZXJ0aWZpY2F0ZSBBdXRob3JpdHmCCQDvohqsVOoqtjAMBgNVHRME\r\n"\
"BTADAQH/MDYGA1UdHwQvMC0wK6ApoCeGJWh0dHA6Ly93d3cuZXhhbXBsZS5jb20v\r\n"\
"ZXhhbXBsZV9jYS5jcmwwDQYJKoZIhvcNAQEFBQADggEBACIxmENPij6zGuTkb4Na\r\n"\
"nQPbx531i6S6WI6fAO/dfKk6AaTz8XW+YWc6xKIXFpu4VtHMoWhPdeTGi1NcbyWM\r\n"\
"FN/3TkpHIoQ0s7J59mg6bFCLFgZiWJRFaANmaQXIee+B5zhW230eCAN4BYgPOgZb\r\n"\
"tYOi9q+1V4QL+A17EgK7JcI08oTcMA1hLDIRLk/E42MGVe/+vn9SXxd1CCaVV0ag\r\n"\
"wJhGyUPHHly74Eb5ezZuPtSNz8b0mFYkghHYK76qTahDVmF1uMCId5FJ+uAvPFPB\r\n"\
"feHC6gSHgdjKjUwU5cteFqnZhGcA+LgvRF8nkb7bzJ/nV4sYdAMFtiVQkY7EhFqK\r\n"\
"AJo=\r\n"\
"-----END CERTIFICATE-----\r\n"\
"\0"\
"\0"


#define WIFI_USER_PRIVATE_KEY_STRING  \
"-----BEGIN RSA PRIVATE KEY-----\r\n"\
"MIIEowIBAAKCAQEArqHulN8vsp5v8iLSHsZqhRJaLwXxkcW5M86v5armBfi2mGGF\r\n"\
"yh8Fhy4FHhqtrA72nsDu34KNWvDOcayzb5pbHfzn9INoZit0ysSCnotCAKwMG0tR\r\n"\
"A80RQOO4p+XQ5w5sQEPE4WHFMOMmXYYFRfGdfpZXQBVL7o+zMaIhBQ508TQNY3Sj\r\n"\
"yjh/3NF6ixummJyt1xOzwKpju4zxqF0mkgDhl0/hBNiK78jfY96pAgF1a0o/INXW\r\n"\
"Vj/9PzRZBMy+PXZ9Zlw+Wap/vbSMLRPbR694LawjnEoG/8MO5AAic4o+nydQweXf\r\n"\
"yGx8wR/t7EPYbNulJKAqZsCcnphuog96tos3BwIDAQABAoIBAC+uWxwezEEy7VHp\r\n"\
"nAF3H4Jx/lm0f77Tg7vXhUNqJCAhRonYptUHH1VHhXIwypaeB0xVVWOFm97mvRdg\r\n"\
"liW0uYg4k4S5lHnBB8SNXzS2xTC1ySBysg4docZb5wFAyIct8vShDFKVhWfRvZnv\r\n"\
"i+ULEXuMxQnkRrU+WykEIYvl5JwurLoab/RcnJGrfwSet1LtkOWLAvu7wfTVIfpd\r\n"\
"Hq4sKOKOrF+/rd18wdsibw+m++sIT3lqipFmo4G3zlBFTQNWmuEDcNI+hIEIkKqc\r\n"\
"xS1A7pkIRNmD0wq5vK2Wcz29lw0gRDtyyB4yYHmGsqrLM6P0MjjTSomvuu+g6Ynx\r\n"\
"5LKfKYECgYEA1OpynHhT7igZ7n+OlQxTvqUlCaiavi8hobn0vosKC3TvuX8EIXEU\r\n"\
"DvOCa288GLSETPgQZ6fRaaMWGZ6s/m78dN/x5sJOkQ5e6T+pYTQxhtbQJpNke2c5\r\n"\
"OjXv6WUvOpFShaNugk2MFZ5XFYc9/XE9dpUraAnf79HyLFlWFSbTc2cCgYEA0fhR\r\n"\
"oQRmoVpp4bXqk5H+qFy1vunehc7tvz4qPn3IIACtBidvU/1sk3AKmuD/goLsXal4\r\n"\
"1fJBht85r+AT8T2WEiT712GThEBNDo1tdKOryGUJ+JpTnH+MVCGzZABaFsGzsoto\r\n"\
"6/yvuP+3iB0temXNiKXd8xOf6j2WP2HExoX2e2ECgYAotedSOcO6jtpeeu9VWnUM\r\n"\
"x+DsFtv3r2Vb2gWNBWLXS/WmXz338vol3bO7PRqdkq6VVP02BKWK8E8CeQ+U2r3U\r\n"\
"zxkeyVpibAAD01NcrbEmmh5UsewNmHAdzEBiPSGpri28NcKkkZVOcOcvy5szwZpB\r\n"\
"+UXCb9jsrxUCG9bU1oap0wKBgQC07VyperlU4CvMqozvjCX7vad+UtfERYLoABE2\r\n"\
"t/vyV6LAYsqzfPRKN4lr2+VP+LDkJFxTPuTn6wIyYsvMXF0sVWINPa93jlDX8VYX\r\n"\
"49DqZOYqqhDSEbZB6KhDy2nVoiPN/gLzFlH0/6SOTVYa2g45aXPPOWHa3ibRhtWg\r\n"\
"m17BAQKBgBMPpyVAFXPGCvHEo6m8x5N/g9Uh/rZ2Q5ak4zr/rP/zH00wEHNw+APA\r\n"\
"3B7W21bonwgV+n8ZVmNmuauBsjA38uPvG5sAxstV3pcP12XrUPxOeA+ZK1t8vFe7\r\n"\
"AB7chbSYpoqnhNljE1OcCC00q1AHIqBN66RDSu8sVtAvyj1aeG05\r\n"\
"-----END RSA PRIVATE KEY-----\r\n"\
"\0"\
"\0"

#define WIFI_USER_CERTIFICATE_STRING  (const uint8_t*)\
"-----BEGIN CERTIFICATE-----\r\n"\
"MIIDzTCCArWgAwIBAgIBAjANBgkqhkiG9w0BAQsFADCBkjELMAkGA1UEBhMCSU4x\r\n"\
"DjAMBgNVBAgTBUluZGlhMRIwEAYDVQQHEwlTb21ld2hlcmUxFTATBgNVBAoTDEV4\r\n"\
"YW1wbGUgSW5jLjEgMB4GCSqGSIb3DQEJARYRYWRtaW5AZXhhbXBsZS5jb20xJjAk\r\n"\
"BgNVBAMTHUV4YW1wbGUgQ2VydGlmaWNhdGUgQXV0aG9yaXR5MB4XDTE3MTAxMDEz\r\n"\
"Mjc0NloXDTE4MTAxMDEzMjc0NlowcDELMAkGA1UEBhMCSU4xDjAMBgNVBAgTBUlu\r\n"\
"ZGlhMRUwEwYDVQQKEwxFeGFtcGxlIEluYy4xGTAXBgNVBAMUEHVzZXJAZXhhbXBs\r\n"\
"ZS5jb20xHzAdBgkqhkiG9w0BCQEWEHVzZXJAZXhhbXBsZS5jb20wggEiMA0GCSqG\r\n"\
"SIb3DQEBAQUAA4IBDwAwggEKAoIBAQCuoe6U3y+ynm/yItIexmqFElovBfGRxbkz\r\n"\
"zq/lquYF+LaYYYXKHwWHLgUeGq2sDvaewO7fgo1a8M5xrLNvmlsd/Of0g2hmK3TK\r\n"\
"xIKei0IArAwbS1EDzRFA47in5dDnDmxAQ8ThYcUw4yZdhgVF8Z1+lldAFUvuj7Mx\r\n"\
"oiEFDnTxNA1jdKPKOH/c0XqLG6aYnK3XE7PAqmO7jPGoXSaSAOGXT+EE2IrvyN9j\r\n"\
"3qkCAXVrSj8g1dZWP/0/NFkEzL49dn1mXD5Zqn+9tIwtE9tHr3gtrCOcSgb/ww7k\r\n"\
"ACJzij6fJ1DB5d/IbHzBH+3sQ9hs26UkoCpmwJyemG6iD3q2izcHAgMBAAGjTzBN\r\n"\
"MBMGA1UdJQQMMAoGCCsGAQUFBwMCMDYGA1UdHwQvMC0wK6ApoCeGJWh0dHA6Ly93\r\n"\
"d3cuZXhhbXBsZS5jb20vZXhhbXBsZV9jYS5jcmwwDQYJKoZIhvcNAQELBQADggEB\r\n"\
"AHdEcZbH+hgdl766gWnSP++RkSxwl9etxmX8Xxhqwzpr4RnqdmQZZBrxiPs4wd2a\r\n"\
"/mTvqbXXv1VMizW2k6XRM6z5AQ3aNvIxzWY2TEaF2GIsi9bBRLI52ASyFB6qring\r\n"\
"DNA726PhIHLkztrsM/BJ2JKfvqZAXJrdyVFT169FmMIYl8B+odtonunzGGrDHL9V\r\n"\
"yGOvh5pG6Gucsj/yzeTy5e0sgHRj0arUSP1Z+ZufCVEJJCHv8nzhcqItnc6yYyxA\r\n"\
"o9Qvh/ScHBg42a4BuClza89QLuM/Qtm3+NH5l6mXGbCeavJlcd5MbhSkhwKAYAMC\r\n"\
"xlAHtFSIAW//MokHpMNnZ+Q=\r\n"\
"-----END CERTIFICATE-----\r\n"\
"\0"\
"\0"

#ifdef __cplusplus
} /*extern "C" */
#endif
