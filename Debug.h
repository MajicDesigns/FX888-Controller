#pragma once

#define DEBUG 0
#define PID_PROFILE 0

#if DEBUG
#define PRINTS(s)   { Serial.print(F(s)); }
#define PRINT(s,v)  { Serial.print(s); Serial.print(v); }
#define PRINTX(s,v) { Serial.print(s); Serial.print(v, HEX); }
#else
#define PRINTS(s)
#define PRINT(s,v)
#define PRINTX(s,v)
#endif

#if PID_PROFILE
#define PID_TRACE_HEADER Serial.print(F("\n, SP, CO, CV"))
#define PID_TRACE(t, sp, co, cv) \
{ \
  Serial.write('\n'); Serial.print(t); \
  Serial.write(','); Serial.print(sp); \
  Serial.write(','); Serial.print(co); \
  Serial.write(','); Serial.print(cv); \
}
#else
#define PID_TRACE_HEADER
#define PID_TRACE(t, sp, co, cv)
#endif
