add_executable(kite_gen kite_gen.cpp)
target_link_libraries(kite_gen PRIVATE kite_sim kite_core)
kite_enable_warnings(kite_gen)
