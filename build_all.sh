cd acado_code_generator/build
make
./mpc
sleep .5
echo "Code generation completed"
mv simple_mpc_export/* ../../acado_mpc_export/
cd ../../acado_mpc_export
make
sleep .5
cd ../build
cmake ..
make
