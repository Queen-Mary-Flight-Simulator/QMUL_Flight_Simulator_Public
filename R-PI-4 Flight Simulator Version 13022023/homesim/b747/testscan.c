#include <stdlib.h>
#include <stdio.h>

#include "scan.h"

int main()
{
    string filenames[100];
	int nfiles;
    int i;
	
	nfiles = ScanDir(".", ".sav", filenames);
    for (i=0; i<nfiles; i+=1)
    {
	    printf("%s\n", filenames[i]);
    }	
	return 0;
}