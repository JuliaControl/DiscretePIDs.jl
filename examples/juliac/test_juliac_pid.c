#include <stdio.h>
#include <stdlib.h>
#include <dlfcn.h>
// #include <julia.h>
#include "juliac_pid.h"

// Path to juliac compiled shared object file
#define LIB_PATH "/home/fredrikb/.julia/dev/DiscretePIDs/examples/juliac/juliac_pid.so" // NOTE: modify this path if needed

int main() {

    // Load the shared library
    printf("Loading juliac_pid.so\n");
    void *lib_handle = dlopen(LIB_PATH, RTLD_LAZY);
    if (!lib_handle) {
        fprintf(stderr, "Error: Unable to load library %s\n", dlerror());
        exit(EXIT_FAILURE);
    }
    printf("Loaded juliac_pid.so\n");

    // Locate the julia functions function
    printf("Finding symbols\n");

    calculate_control_t calculate_control = (calculate_control_t) dlsym(lib_handle, "calculate_control!");
    set_K_t                         set_K = (set_K_t)             dlsym(lib_handle, "set_K!");
    set_Ti_t                       set_Ti = (set_Ti_t)            dlsym(lib_handle, "set_Ti!");
    set_Td_t                       set_Td = (set_Td_t)            dlsym(lib_handle, "set_Td!");
    reset_state_t             reset_state = (reset_state_t)       dlsym(lib_handle, "reset_state!");


    if (calculate_control == NULL) {
        char *error = dlerror();
        fprintf(stderr, "Error: Unable to find symbol: %s\n", error);
        exit(EXIT_FAILURE);
    }
    printf("Found all symbols!\n");

    // Trivial test program that computes a few control outputs and modifies K
    double r = 1.0, y = 0.0, uff = 0.0;
    double result = calculate_control(r, y, uff);
    printf("calculate_control! returned: %f\n", result);
    result = calculate_control(r, y, uff);
    printf("calculate_control! returned: %f\n", result);
    set_K(0.0, r, y);
    for (int i = 0; i < 3; ++i) {
        result = calculate_control(r, y, uff);
        printf("calculate_control! returned: %f\n", result);
    }

    return 0;
}


// Compile this C program using a command like:
// gcc -o pid_program test_juliac_pid.c -I $HOME/.julia/juliaup/julia-1.12.1+0.x64.linux.gnu/include/julia -L$HOME/.julia/juliaup/julia-1.12.1+0.x64.linux.gnu/lib -ljulia -ldl
// (modify the Julia path to match your installation)
//
// Run with:
// export LD_LIBRARY_PATH=$HOME/.julia/juliaup/julia-1.12.1+0.x64.linux.gnu/lib:$HOME/.julia/juliaup/julia-1.12.1+0.x64.linux.gnu/lib/julia:$LD_LIBRARY_PATH
// ./pid_program