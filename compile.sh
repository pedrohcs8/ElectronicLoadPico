rm -rf build/*
cd build
cmake ..
bear -- make
cp electronic_load_pico.uf2 /media/pedrohcs8/RPI-RP2
$shell
