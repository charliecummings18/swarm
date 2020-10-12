//
// mpi_hello.c
//
// Simple "hello world" demo
//
#include <stdlib.h>
#include <stdio.h>
#include <mpi.h>

int main(){
    int rank, size, length;
    char name[MPI_MAX_PROCESSOR_NAME];
    
    MPI_Init(NULL,NULL);
    MPI_Comm_size(MPI_COMM_WORLD,&size);
    MPI_Comm_rank(MPI_COMM_WORLD,&rank);
    MPI_Get_processor_name(name, &length);
    printf("Hello, from process (rank) %d of %d, running on processor %s\n",rank,size,name);
    MPI_Finalize();
}
