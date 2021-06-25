//COLOCAR PONTO E VÍRGULA ENTRE ARGUMENTOS DE FOR E COLOCAR . FLUTUANTE NAS MATRIZES DE DOUBLE DECLARADAS
//COLOCAR PONTO E VÍRGULA ENTRE ARGUMENTOS DE FOR E COLOCAR . FLUTUANTE NAS MATRIZES DE DOUBLE DECLARADAS
//COLOCAR PONTO E VÍRGULA ENTRE ARGUMENTOS DE FOR E COLOCAR . FLUTUANTE NAS MATRIZES DE DOUBLE DECLARADAS
//COLOCAR PONTO E VÍRGULA ENTRE ARGUMENTOS DE FOR E COLOCAR . FLUTUANTE NAS MATRIZES DE DOUBLE DECLARADAS
//COLOCAR PONTO E VÍRGULA ENTRE ARGUMENTOS DE FOR E COLOCAR . FLUTUANTE NAS MATRIZES DE DOUBLE DECLARADAS
//COLOCAR PONTO E VÍRGULA ENTRE ARGUMENTOS DE FOR E COLOCAR . FLUTUANTE NAS MATRIZES DE DOUBLE DECLARADAS
//COLOCAR PONTO E VÍRGULA ENTRE ARGUMENTOS DE FOR E COLOCAR . FLUTUANTE NAS MATRIZES DE DOUBLE DECLARADAS
//COLOCAR PONTO E VÍRGULA ENTRE ARGUMENTOS DE FOR E COLOCAR . FLUTUANTE NAS MATRIZES DE DOUBLE DECLARADAS
//COLOCAR PONTO E VÍRGULA ENTRE ARGUMENTOS DE FOR E COLOCAR . FLUTUANTE NAS MATRIZES DE DOUBLE DECLARADAS
//COLOCAR PONTO E VÍRGULA ENTRE ARGUMENTOS DE FOR E COLOCAR . FLUTUANTE NAS MATRIZES DE DOUBLE DECLARADAS

//#include <stdio.h>
//#include <math.h>
//#include <bits/stdc++.h>

#include <iostream>
#include <fstream>
#include <cstdlib>
#include <cmath>
//#include <bits/stdc++.h>

using namespace std;

// Declaração só para testes, os dados virão de  outro código:
double ax, ay, az;
double gx, gy, gz;
double mx, my, mz;
double x = 0.0, y = 0.0, ze = 0.0, r = 0.0, p = 0.0, ya = 0.0;
double vx, vy, vz, vr, vp, vya;
double accx, accy, accz, accr, accp, accya;


//pontos sigma
double t; //intervalo de tempo do filtro de Kalman
double alfa = 1e-3, ki = 0, lambda = 0;
int beta = 2, n = 12, i, j, j_1, k;
double mu[12][1] = {{0.0},{0.0},{0.0},{0.0},{0.0},{0.0},{0.0},{0.0},{0.0},{0.0},{0.0},{0.0}}; //vetor média de cada estado
double mu_0[12][1] = {{0.0},{0.0},{0.0},{0.0},{0.0},{0.0},{0.0},{0.0},{0.0},{0.0},{0.0},{0.0}}; //vetor média de cada estado no passo anterior

double X_m0[12][25]; //matriz auxiliar, armazena f[coluna]-mu
double X_m1[25][12]; //matriz auxiliar de covariância (prediction step)
double cov[12][1]; //vetor de covariância do prediction step (colunas de X_m1 somadas)
double cov_f[12][1];
double X_m3[12][25]; //matriz auxiliar, armazena M*a (para sigma points)
double obs_sigma[12][25]; //matriz de observação, armazena M*a+N*g (para sigma points) e depois a observação
double X_m4[12][1]; //vetor auxiliar, armazena M*a (para último estado)
double obs[12][1]; //vetor de observação, armazena M*a+N*g (para último estado) e depois a observação
double z[12][1]; //soma ponderada das colunas de obs por seus respectivos pesos
double X_m6[12][25]; //matriz auxiliar, armazena obs[coluna]-z
double X_m7[12][25]; //matrix auxiliar, armazena peso*(obs[coluna]-z)(transposta(obs[coluna]-z)
double S_t[12][1]; //soma das colunas de X_m7
double X_m9[12][25]; //matriz auxiliar, armazena (sigma-mu)
double X_m10[12][25]; //matriz auxiliar, armazena (obs-z)
double X_m11[12][25]; //matriz auxiliar, armazena (sigma-mu)*transposta(obs-z)
double cov_cruz[12][1]; //vetor covariância cruzada, update step, soma das colunas de X_m11
double X_m12[12][1]; //vetor auxiliar, armazena obs-z
double X_m13[12][1]; //vetor auxiliar, armazena K_t*X_m12
double mu_f[12][1]; //vetor de média ao final do ciclo de Kalman
double X_m14[12][12]; //matriz auxiliar, armazena K_t*S_t*K_t_transposta
double X_m15[12][1] = {{X_m14[0][0]}, {X_m14[1][1]}, {X_m14[2][2]}, {X_m14[3][3]}, {X_m14[4][4]}, {X_m14[5][5]}, {X_m14[6][6]}, {X_m14[7][7]}, {X_m14[8][8]}, {X_m14[9][9]}, {X_m14[10][10]}, {X_m14[11][11]}}; //vetor auxiliar, armazena 

double a[3][0]; //acelerações medidas (R*(ac-gr))

double ac[3][1] = {{ax}, {ay}, {az}};  //acelerômetro
double gi[3][1] = {{gx}, {gy}, {gz}};  //giroscópio
double m[3][1] = {{mx}, {my}, {mz},}; //magnetômetro

double R[3][3] = {{cos(mx)*cos(mz), sin(mx)*sin(my)*cos(mz)-cos(mx)*sin(mz), cos(mx)*sin(my)*cos(mz)+sin(mx)*sin(mz)}, //Earth-fixed to body frame
                  {sin(my)*sin(mz), sin(mx)*sin(my)*sin(mz)+cos(mx)*cos(mz), cos(mx)*sin(my)*sin(mz)-sin(mx)*cos(mz)}, 
                  {-sin(my), sin(mx)*cos(my), cos(mx)*cos(my)}}; 

double gr[3][1] = {{0.0}, {0.0}, {-9.81}}; //vetor gravidade 

double M[12][3] = {{t*t/2, 0.0, 0.0}, //matriz acelerômetro
                   {0.0, t*t/2, 0.0}, 
                   {0.0, 0.0, t*t/2}, 
                   {0.0, 0.0, 0.0}, 
                   {0.0, 0.0, 0.0},
                   {0.0, 0.0, 0.0},
                   {t, 0.0, 0.0},
                   {0.0, t, 0.0},
                   {0.0, 0.0, t}, 
                   {0.0, 0.0, 0.0},
                   {0.0, 0.0, 0.0},
                   {0.0, 0.0, 0.0}}; 

double N[12][3] = {{0.0, 0.0, 0.0}, //matriz giroscópio
                   {0.0, 0.0, 0.0},
                   {0.0, 0.0, 0.0},
                   {t, 0.0, 0.0},
                   {0.0, t, 0.0}, 
                   {0.0, 0.0, t}, 
                   {0.0, 0.0, 0.0},
                   {0.0, 0.0, 0.0},
                   {0.0, 0.0, 0.0},
                   {1.0, 0.0, 0.0},
                   {0.0, 1.0, 0.0},
                   {0.0, 0.0, 1.0}}; 
            
double X[12][1] = {{x}, {y}, {ze}, {r}, {p}, {ya}, {vx}, {vy}, {vz}, {vr}, {vp}, {vya}}; //vetor de estados

double A[12][12] = {{1.0, 0.0, 0.0, 0.0, 0.0, 0.0, t, 0.0, 0.0, 0.0, 0.0, 0.0}, //matriz de estados
                    {0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, t, 0.0, 0.0, 0.0, 0.0}, 
                    {0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, t, 0.0, 0.0, 0.0},
                    {0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, t, 0.0, 0.0}, 
                    {0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, t, 0.0},
                    {0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, t}, 
                    {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0}, 
                    {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0}, 
                    {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0}, 
                    {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0}, 
                    {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0}, 
                    {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0}};

double u[6][1] = {{accx}, {accy}, {accz}, {accr}, {accp}, {accya}}; //vetor de controle

double B[12][6] = {{t*t/2, 0.0, 0.0, 0.0, 0.0, 0.0}, //matriz de controle
                   {0.0, t*t/2, 0.0, 0.0, 0.0, 0.0}, 
                   {0.0, 0.0, t*t/2, 0.0, 0.0, 0.0}, 
                   {0.0, 0.0, 0.0, t*t/2, 0.0, 0.0}, 
                   {0.0, 0.0, 0.0, 0.0, t*t/2, 0.0}, 
                   {0.0, 0.0, 0.0, 0.0, 0.0, t*t/2}, 
                   {t, 0.0, 0.0, 0.0, 0.0, 0.0}, 
                   {0.0, t, 0.0, 0.0, 0.0, 0.0}, 
                   {0.0, 0.0, t, 0.0, 0.0, 0.0}, 
                   {0.0, 0.0, 0.0, t, 0.0, 0.0},
                   {0.0, 0.0, 0.0, 0.0, t, 0.0}, 
                   {0.0, 0.0, 0.0, 0.0, 0.0, t}};

double lower[12][12]; //para decomposição de Cholesky

double sigma[12][25]; //matriz pontos Sigma

double weight_m[25][1]; //vetor de pesos dos pontos sigma para média
double weight_c[25][1]; //vetor de pesos de pontos sigma para covariância

double g[12][0]; //vetor armazena B*u
double f[12][25]; //matriz armazena A*sigma, cada coluna é um vetor de estados

double S_t_diag[12][12] = {{S_t[0][0], 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
                           {0.0, S_t[1][0], 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
                           {0.0, 0.0, S_t[2][0], 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
                           {0.0, 0.0, 0.0, S_t[3][0], 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
                           {0.0, 0.0, 0.0, 0.0, S_t[4][0], 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
                           {0.0, 0.0, 0.0, 0.0, 0.0, S_t[5][0], 0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
                           {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, S_t[6][0], 0.0, 0.0, 0.0, 0.0, 0.0},
                           {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, S_t[7][0], 0.0, 0.0, 0.0, 0.0},
                           {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, S_t[8][0], 0.0, 0.0, 0.0},
                           {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, S_t[9][0], 0.0, 0.0},
                           {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, S_t[10][0], 0.0},
                           {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, S_t[11][0]}};

double S_t_diag_inverse[12][12] = {{1.0/S_t[0][0], 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
                                  {0.0, 1.0/S_t[1][0], 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
                                  {0.0, 0.0, 1.0/S_t[2][0], 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
                                  {0.0, 0.0, 0.0, 1.0/S_t[3][0], 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
                                  {0.0, 0.0, 0.0, 0.0, 1.0/S_t[4][0], 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
                                  {0.0, 0.0, 0.0, 0.0, 0.0, 1.0/S_t[5][0], 0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
                                  {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0/S_t[6][0], 0.0, 0.0, 0.0, 0.0, 0.0},
                                  {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0/S_t[7][0], 0.0, 0.0, 0.0, 0.0},
                                  {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0/S_t[8][0], 0.0, 0.0, 0.0},
                                  {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0/S_t[9][0], 0.0, 0.0},
                                  {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0/S_t[10][0], 0.0},
                                  {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0/S_t[11][0]}};

double cov_cruz_diag[12][12] = {{cov_cruz[0][0], 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
                                {0.0, cov_cruz[1][0], 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
                                {0.0, 0.0, cov_cruz[2][0], 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
                                {0.0, 0.0, 0.0, cov_cruz[3][0], 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
                                {0.0, 0.0, 0.0, 0.0, cov_cruz[4][0], 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
                                {0.0, 0.0, 0.0, 0.0, 0.0, cov_cruz[5][0], 0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
                                {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, cov_cruz[6][0], 0.0, 0.0, 0.0, 0.0, 0.0},
                                {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, cov_cruz[7][0], 0.0, 0.0, 0.0, 0.0},
                                {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, cov_cruz[8][0], 0.0, 0.0, 0.0},
                                {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, cov_cruz[9][0], 0.0, 0.0},
                                {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, cov_cruz[10][0], 0.0},
                                {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, cov_cruz[11][0]}};

double K_t[12][12];
double K_t_transp[12][12];



void Cholesky_Decomposition(double matrix[12][12])
{
    // Decomposing a matrix into Lower Triangular
    for (int i = 0; i < 12; i++) 
    {
        for (int j = 0; j <= i; j++) 
        {
            double sum = 0;
 
            if (j == i) // summation for diagnols
            {
                for (int k = 0; k < j; k++)
                    sum += pow(lower[j][k], 2);
                lower[j][j] = sqrt(matrix[j][j] - sum);
            } 
            else 
            {
                // Evaluating L(i; j) using L(j; j)
                for (int k = 0; k < j; k++)
                    sum += (lower[i][k] * lower[j][k]);
                lower[i][j] = (matrix[i][j] - sum) / lower[j][j];
            }
        }
    }
}

//determinação dos pontos sigma
void sigma_points (void)
{           
            //! PROBLEMAS COM O "P" E O "P_" QUE NÃO FORAM DECLARADOS ANTERIORMENTE
            
            lambda = alfa*alfa*(n + ki) - n;
            P_ = (n + lambda)*P;
            Cholesky_Decomposition(P_);
            double sum;
            //mu_0 = mu_f; //atribuição para não perder as médias do passo anterior
            for (i=0; i<12; i++) //a primeira coluna da matriz sigma é a coluna de médias anteriores
            {
                        mu_0[i][0] = mu_f[i][0];
                        sigma[i][0] = mu_f[i][0];
            }
            
            for (i=1; i<13; i++) //determinação dos pontos sigma restantes
            {
                        for (j=0; j<12; j++)
                        {
                                    sigma[j][i] = mu_0[i][0] + lower[j][i];
                                    sigma[j][i+12] = mu_0[i][0] - lower[j][i];
                        }
            }
            
            //Pesos dos pontos sigma
            weight_m[0][0] = lambda/(n+lambda);
            weight_c[0][0] = weight_m[0][0] + (1 + alfa*alfa + beta);  
            
            for (i=1; i<25; i++)
            {
                        weight_m[i][0] = 1/2*(n+lambda);
                        weight_c[i][0] = weight_m[i][0];
            }
}

int main (void){

  double sum;
  //ac = (ac-gr)
  for (i=0; i<3; i++)
  {
    ac[i][0] = ac[i][0]-gr[i][0];
  }
  //ac = R*ac
  for (i = 0; i<3; i++)
  {
    sum = 0;
    for (j=0; j<3; j++)
    {
      sum = sum + R[i][j]*ac[j][0];
    }
    a[i][0] = sum;
  }
  
  //determinação dos pontos sigma
  sigma_points();
  
  //PREDICTION STEP
  //f(sigma;u)
  //B*u
  for (i=0; i<12; i++)
  {
    sum=0;
    for (j=0; j<6; j++)
    {
      sum = sum + B[i][j]*u[j][0];
    }
    g[i][0] = sum;
  }
  
  //A*sigma[coluna]
  for (i=0; i<25; i++)
  {
    for (j=0; j<12; j++)
    {
      sum = 0;
      for (k=0; k<12; k++)
      {
        sum = sum + A[k][j]*sigma[k][i];
      }
      f[j][i] = sum;
    }
  }
  
  //f[coluna] + B*u
  for (i=0; i<25; i++)
  {
    for (j<0; j<12; j++)
    {
      f[j][i] = f[j][i] + g[i][0];
    }
  }
  
  //covariância
  //peso*(f[coluna]-mu)(transposta(f[coluna]-mu)
  for (i=0; i<25; i++)
  {
    for (j=0; j<12; j++)
    {
      X_m0[j][i] = f[j][i] - mu[j][0]; //X_m0 é apenas matriz auxiliar [12][25]
    }
    for (j_1=0; j_1<12; j_1++)
    {
      sum = 0;
      for (k=0; k<12; k++)
      {
        sum = sum + X_m0[j][i]*X_m0[i][k];
      }
      X_m1[i][j] = weight_c[i][0]*sum; //X_m1 é matriz auxiliar de covariância [25][12]       
    }
  }
  
  //soma das colunas X_m1
  for (i=0; i<25; i++)
  {
    for (j=0; j<12; j++)
    {
      cov[j][0] = cov[j][0] + X_m1[j][i]; //cov[12][1] vetor de covariância [12][1]; ainda é necessário implementar a soma com Q
    }
  }
  
  //cálculo média
  //mu_0 = mu;
  for (i = 0; i < 12; i++){
    mu_0[i][0] = mu[i][0];
  }
  
  for (i=0; i<25; i++)
  {
    for (j=0; j<12; j++)
    {
      mu[j][0] = mu[j][0] + weight_m[i][0]*X[j][i];
    }
  }
  
  //UPDATE STEP
  //Modelo de observação com pontos sigma
  //M*a
  for (i=0; i<25; i++)
  {
    for (j=0; j<12; j++)
    {
      sum = 0;
      for (k=0; k<3; k++)
      {
        sum = sum + M[j][k]*a[k][0];
      }
      X_m3[j][i] = sum; //X_m3 é apenas matriz auxiliar [12][25]
    }
  }
  
  //N*g
  for (i=0; i<25; i++)
  {
    for (j=0; j<12; j++)
    {
      sum = 0;
      for (k=0; k<3; k++)
      {
        sum = sum + N[j][k]*g[k][0];
      }
      obs_sigma[j][i] = X_m3[j][i] + sum; //obs é matriz de observação [12][25]
    }
  }
  
  //observação de sigma
  for (i=0; i<12; i++)
  {
    for (j=0; j<25; j++)
    {
      obs_sigma[i][j] = obs_sigma[i][j] + sigma[i][j];
    }
  }
  
  //Modelo de observação com últimos estados
  //M*a
  for (i=0; i<12; i++)
  {
    for (j=0; j<1; j++)
    {
      sum = 0;
      for (k=0; k<3; k++)
      {
        sum = sum + M[j][k]*a[k][j];
      }
      X_m4[i][j] = sum; //X_m3 é apenas matriz auxiliar [12][1]
    }
  }
  
  //N*g
  for (i=0; i<12; i++)
  {
    for (j=0; j<1; j++)
    {
      sum = 0;
      for (k=0; k<3; k++)
      {
        sum = sum + N[j][k]*g[k][j];
      }
      obs[i][j] = X_m4[i][j] + sum; //obs é vetor de observação [12][1]
    }
  }
  
  //observação dos últimos estados
  for (i=0; i<12; i++)
  {
    for (j=0; j<12; j++)
    {
      obs[i][j] = obs[i][j] + X[i][j];
    }
  }
  
  //cálculo da soma das colunas de obs; multiplicadas por seus respectivos pesos
  for (i=0; i<12; i++)
  {
    for (j=0; j<25; j++)
    {
      z[i][0] = obs[i][j]*weight_m[j][0]; //z é vetor auxiliar [12][1]; a declaração está errada
    }
  }
  
  //peso*(obs[coluna]-z)(transposta(obs[coluna]-z)
  for (i=0; i<25; i++)
  {
    for (j=0; j<12; j++)
    {
      X_m6[j][i] = obs[j][i] - z[j][0]; //X_m6 é apenas matriz auxiliar [12][25]
    }
    for (j_1=0; j_1<12; j_1++)
    {
      sum = 0;
      for (k=0; k<12; k++)
      {
        sum = sum + X_m6[j][i]*X_m6[i][k];
      }
      X_m7[i][j] = weight_c[i][0]*sum; //X_m7 é apenas matriz auxiliar [12][25]
    }
  }
  
  //soma das colunas X_m7
  for (i=0; i<25; i++)
  {
    for (j=0; j<12; j++)
    {
      S_t[j][0] = S_t[j][0] + X_m7[j][i]; //S_t vetor [12][1]; ainda é necessário implementar a soma com R_t
    }
  }
  
  //covariância cruzada
  //(sigma-mu)*transposta(obs-z)
  for (i=0; i<25; i++)
  {
    for (j=0; j<12; j++)
    {
      X_m9[j][i] = sigma[j][i] - mu[j][0];  //X_m9 e X_m10 são matrizes auxiliares [12][25]
      X_m10[j][i] = obs[j][i] - z[j][0];
    }
    for (j=0; j<12; j++)
    {
      sum = 0;
      for (k=0; k<12; k++)
      {
        sum = sum + X_m9[j][i]*X_m10[i][k];
      }
      X_m11[i][j] = weight_c[i][0]*sum; //X_m11 é apenas matriz auxiliar [12][25]
    }
  }
  
  //soma das colunas X_m11
  for (i=0; i<25; i++)
  {
    for (j=0; j<12; j++)
    {
      cov_cruz[j][0] = cov_cruz[j][0] + X_m11[j][i]; //vetor cov_cruz [12][1]
    }
  }
  
  //ganho de Kalman
  for (i=0; i<12; i++)
  {
    for (j=0; j<12; j++)
    {
      for (k=0; k<12; k++)
      {
        K_t[i][j] = K_t[i][j] + cov_cruz_diag[i][k]*S_t_diag_inverse[k][j];
      }
    }
  }
  
  //subtração entre vetor observação com últimos estados e vetor z
  for (i=0; i<12; i++)
  {
    for (j=0; j<1; j++)
    {
      X_m12[i][j] = obs[i][j] - z[i][j];
    }
  }
  
  //multiplicação K_t*X_m12
  for (i=0; i<12; i++)
  {
    for (j=0; j<1; j++)
    {
      for (k=0; k<12; k++)
      {
        X_m13[i][j] = X_m13[i][j] + K_t[i][k]*X_m12[k][j]; //X_m13 é vetor auxiliar [12][3]
      }
    }
  }
  
  //soma X_m13 + mu; resultando na média final
  for (i=0; i<12; i++)
  {
    for (j=0; j<1; j++)
    {
      mu_f[i][j] = X_m13[i][j] + mu[i][j]; //mu_f é a média ao final do ciclo do Filtro de Kalman Unscented
      X[i][j] = mu_f[i][j];
    }
  }
  
  //K_t*S_t*K_t_transposta (K_t_transposta = K_t; pois são diagonais)
  //S_t*K_t_transposta
  for (i=0; i<12; i++)
  {
    for (j=0; j<12; j++)
    {
      for (k=0; k<12; k++)
      {
        X_m14[i][j] = X_m14[i][j] + S_t[i][k]*K_t[k][j];
      }
    }
  }
  
  //K_t*S_t
  for (i=0; i<12; i++)
  {
    for (j=0; j<12; j++)
    {
      for (k=0; k<12; k++)
      {
        X_m14[i][j] = X_m14[i][j] + K_t[i][k]*S_t[k][j];
      }
    }
  }
  
  //covariância final
  for (i=0; i<12; i++)
  {
    for (j=0; j<1; j++)
    {
      cov_f[i][j] = cov[i][j] - X_m15[i][j];
    }
  }
}



/*calcula a média de 5 vetores de estado (objetivo é usar os últimos 5)
double media(double _0[12][1]; double _1[12][1]; double _2[12][1]; double _3[12][1]; double _4[12][1])
{
            return (_0[i][1] + _1[i][1] + _2[i][1] + _3[i][1] + _4[i][1])/5.0;
}
//calcula a covariancia entre 5 vetores de estado (objetivo é usar os últimos 5)
double covariancia(double media[12][1]; double _0[12][1]; double _1[12][1]; double _2[12][1]; double _3[12][1]; double _4[12][1])
{
            int i; j; k;
            double media_m[12][5];
            double sum;
            double covar [12][12];
            
            for (i=0; i<12; i++)
            {
                        media_m[i][0] = media[i][1] - _0[i][1];
                        
            }
            
            for (i=0; i<12; i++)
            {
                        media_m[i][1] = media[i][1] - _1[i][1];
                        
            }
            
            for (i=0; i<12; i++)
            {
                        media_m[i][2] = media[i][1] - _2[i][1];
                        
            }
            
            for (i=0; i<12; i++)
            {
                        media_m[i][3] = media[i][1] - _3[i][1];
                        
            }
            
            for (i=0; i<12; i++)
            {
                        media_m[i][4] = media[i][1] - _4[i][1];
                        
            }
            
            for (i=0; i<12; i++)
            {
                        for (j=i; j<12; j++)
                        {
                                    for (k=0; k<5; k++)
                                    {
                                                sum = sum + media_m[i][k]*media_m[j][k];
                                    }
                                    covar[i][j] =  sum/4.0;
                                    covar[j][i] = covar[i][j];
                        }
                        
            }
       
            return covar;
}
*/

/*
roll_quad  = math.atan(2*(q2*q3 + q0*q1); q0^2 - q1^2 - q2^2 - q3^2) -- Roll
pitch_quad = math.asin(-2*(q1*q3 - q0*q2))                           -- Pitch
yaw_quad   = math.atan(2*(q1*q2 + q0*q3); q0^2 + q1^2 - q2^2 - q3^2) -- Yaw
*/
