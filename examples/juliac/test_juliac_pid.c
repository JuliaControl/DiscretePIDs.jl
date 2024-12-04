#include <stdio.h>
#include <stdlib.h>
#include <dlfcn.h>
// #include <julia.h>

// Path to julia binary folder
#define JULIA_PATH "/home/fredrikb/repos/julia/usr/bin/" // NOTE: modify this path

// Path to juliac compiled shared object file
#define LIB_PATH "/home/fredrikb/.julia/dev/DiscretePIDs/examples/juliac/juliac_pid.so" // NOTE: modify this path



// Define the types of the julia @ccallable functions
typedef void (*jl_init_with_image_t)(const char *bindir, const char *sysimage);
typedef double (*calculate_control_t)(double r, double y, double uff);
typedef void (*set_K_t)(double K, double r, double y);
typedef void (*set_Ti_t)(double Ti);
typedef void (*set_Td_t)(double Td);
typedef void (*reset_state_t)();


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
    jl_init_with_image_t jl_init_with_image = (jl_init_with_image_t)dlsym(lib_handle, "jl_init_with_image");

    calculate_control_t calculate_control = (calculate_control_t)dlsym(lib_handle, "calculate_control!");
    set_K_t set_K = (set_K_t)dlsym(lib_handle, "set_K!");
    set_Ti_t set_Ti = (set_Ti_t)dlsym(lib_handle, "set_Ti!");
    set_Td_t set_Td = (set_Td_t)dlsym(lib_handle, "set_Td!");
    reset_state_t reset_state = (reset_state_t)dlsym(lib_handle, "reset_state!");


    if (jl_init_with_image == NULL || calculate_control == NULL) {
        char *error = dlerror();
        fprintf(stderr, "Error: Unable to find symbol: %s\n", error);
        exit(EXIT_FAILURE);
    }
    printf("Found all symbols!\n");

    // Init julia
    jl_init_with_image(JULIA_PATH, LIB_PATH);

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

    // jl_atexit_hook(0);
    return 0;
}


// Compile this C program using a command like the one above, modified to suit your paths
// export LD_LIBRARY_PATH=/home/fredrikb/repos/julia/usr/lib:$LD_LIBRARY_PATH
// gcc -o pid_program test_juliac_pid.c -I /home/fredrikb/repos/julia/usr/include/julia -L/home/fredrikb/repos/julia/usr/lib -ljulia -ldl
