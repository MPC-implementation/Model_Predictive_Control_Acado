cd acado_code_generator/build
make
./mpc
mv simple_mpc_export/* ../../acado_mpc_export/
cd ../../acado_mpc_export
make
cd ../build
make
