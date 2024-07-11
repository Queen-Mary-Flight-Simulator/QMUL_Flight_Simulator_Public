#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>

#include <SIM/dted.h>

DTED_Record DTED;
double      *Posts;

void LoadDTED(char Filename[]);

/* ---------------------------------- */
int main(int argc, char *argv[])
{
  if (argc > 1) 
  {
    LoadDTED(argv[1]);
  }
  return 0;
}

/* ------------------------------------------------------------------ */
void LoadDTED(char Filename[])
{
    char fname[50];
    FILE *f;
    int dsize;
	int i, j;
	int p;
	
    strcpy(fname, Filename);
    strcat(fname, ".dtd");
    
    if ((f = fopen(fname, "rb")) == NULL)
    {
        printf("Error opening DTED file %s\n", fname);
        return;
    }

    fread(&DTED, sizeof(DTED), 1, f);
    
    dsize = DTED.xPosts * DTED.yPosts * sizeof(double);
    
    if (Posts != NULL)  /* check and remove any loaded DTED */
    {
        free((double *) Posts);
    }

    Posts = (double *) malloc(dsize);
    if (!Posts)
    {
        printf("Error: Insufficient memory for DTED (%d K bytes)\n", dsize / 1000);
        exit(1);
    }
    //printf("DTED_LoadDTED: Malloc: Posts=%p\n", Posts); // ***
    fread(Posts, dsize, 1, f);

    fclose(f);
	
    printf("DTED_LoadDTED: bx1=%f by1=%f bx2=%f by2=%f xPosts=%d yPosts=%d\n",   // ***
           DTED.bx1, DTED.by1, DTED.bx2, DTED.by2, DTED.xPosts, DTED.yPosts);
	
//    for (i=1; i<=DTED.xPosts * DTED.yPosts; i+=1)
//        printf("%f\n", Posts[i]);	   

    p = 0;
	
    for (i=0; i<DTED.xPosts; i+=1)
    {
        for (j=0; j<DTED.yPosts; j+=1)
        {
            float x = DTED.bx1 + (float) (i * 10);
            float y = DTED.by1 + (float) (j * 10);
			
			printf("%f %f %f\n", x/100000.0, y/100000.0, Posts[p]);
            p += 1;
        }
    }

	free(Posts);
}
