git rev-parse --short HEAD > githash.h
set /p GITHASH=<githash.h

echo #ifdef NDEBUG>githash.h
echo     #define VERSION "%GITHASH%-release">>githash.h
echo #else>>githash.h
echo     #define VERSION "%GITHASH%-debug">>githash.h
echo #endif>>githash.h
