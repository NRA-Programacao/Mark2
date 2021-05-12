#include <stdio.h>
#include <math.h>
#include <bits/stdc++.h>

using namespace std;

double sum;
double t = 2;
double A[2] = {8.0, 9.0};
                    
double N[2] = {5.0, 7.0}; 
      
//testando a função multip_matriz                  
int main(){
	double multip_matriz( double **X, int a, int b, double **Y, int c, int d )
	double *matriz = multip_matriz( N, 3, 2, A, 2, 1);
	
	for (int i=0; i<3; i++)
	  {
	              for (int j=0; j<1; j++)
	              {
	              	printf("%d", matriz[i][j] );
	        	}
	            printf("/n");  
		}
	
}

//X[a][b]*Y[c][d] = Z[a][d]
double multip_matriz( double **X, int a, int b, double **Y, int c, int d )
{
	double **Z = (double**)malloc(a*d*sizeof(double));
	if(b==c){
	  for (int i=0; i<d; i++)
	  {
	              for (int j=0; j<b; j++)
	              {
	                          sum = 0;
	                          for (int k=0; k<a; k++)
	                          {
	                                      sum = sum + X[k][j]*Y[j][i];
	                          }
	                          Z[j][i] = sum;
	   }
	   }
			}
			
	else{
		return 0;
	}
	
	return **Z;
}

//X[12][12]*Y[12][25] = Z[12][25]
double multiplica12_25( double X[12][12], double Y[12][25], double Z[12][25])
{
  for (int i=0; i<25; i++)
  {
              for (int j=0; j<12; j++)
              {
                          sum = 0;
                          for (int k=0; k<12; k++)
                          {
                                      sum = sum + X[k][j]*Y[k][i];
                          }
                          Z[j][i] = sum;
   }
   }
}

//soma das colunas de X
double somacoluna( double X[12][1], double Y[12][1])
{
    for (int i=0; i<25; i++)
    {
                for (int j=0; j<12; j++)
                {
                            Y[j][0] = Y[j][0] + X[j][i];
                }
    }
}

 //X[12][3]*Y[3][1];
 double multiplica12_25( double X[12][3], double Y[3][1], double Z[12][25]){
     for (int i=0; i<25; i++)
     {
                for (int j=0; j<12; j++)
                {
                            sum = 0;
                            for (int k=0; k<3; k++)
                            {
                                        sum = sum + X[j][k]*Y[k][0];
                            }
                            Z[j][i] = sum;
                }

     }
 }
 
 //Matriz Transposta
 double transposta12_25( double matriz[25][12], double transposta[12][25]){
 double aux;
  for (int i = 0; i<25; i++) //igualando toda a matriz a 0
  {
    for (int j = 0; j<12; j++)
    {
      matriz[i][j] = 0;
    }
  }
  for (int i = 0; i < 25; i++)
  {
    for (int j = 0; j < 12; j++)
    {
        aux = matriz[i][j];
        matriz[i][j] = matriz[j][i];
        matriz[j][i] = aux;
      }
    }

  for (int i = 0; i < 12; i++)
  {
    for (int j = 0; j < 25; j++)
    {
      transposta[i][j] = matriz[i][j];
    }
  }
}
  
 double transposta25_12( double matriz[25][12], double transposta[12][25]){
 double aux;
  for (int i = 0; i<25; i++) //igualando toda a matriz a 0
  {
    for (int j = 0; j<12; j++)
    {
      matriz[i][j] = 0;
    }
  }
  for (int i = 0; i < 25; i++)
  {
    for (int j = 0; j < 12; j++)
    {
        aux = matriz[i][j];
        matriz[i][j] = matriz[j][i];
        matriz[j][i] = aux;
      }
    }
  

  for (int i = 0; i < 12; i++)
  {
    for (int j = 0; j < 25; j++)
    {
      transposta[i][j] = matriz[i][j];
    }
  }
}
