#include <stdio.h>
#include <math.h>
#include <bits/stdc++.h>

using namespace std;

double t; //intervalo de tempo do filtro de Kalman
int alfa = 1e-3, ki = 0, beta = 2, n = 12, i, j;
double mu = 0;

double X_m0[12][1] = {{0}, {0}, {0}, {0}, {0}, {0}, {0}, {0}, {0}, {0}, {0}, {0}}; //estados medidos k
double X_m1[12][1] = {{0}, {0}, {0}, {0}, {0}, {0}, {0}, {0}, {0}, {0}, {0}, {0}}; //estados medidos k-1
double X_m2[12][1] = {{0}, {0}, {0}, {0}, {0}, {0}, {0}, {0}, {0}, {0}, {0}, {0}}; //estados medidos k-2
double X_m3[12][1] = {{0}, {0}, {0}, {0}, {0}, {0}, {0}, {0}, {0}, {0}, {0}, {0}}; //estados medidos k-3
double X_m4[12][1] = {{0}, {0}, {0}, {0}, {0}, {0}, {0}, {0}, {0}, {0}, {0}, {0}}; //estados medidos k-4
double a[3][1] = {{ax}, {ay}, {az}};  //acelerômetro
double g[3][1] = {{gx}, {gy}, {gz}};  //giroscópio
double m[3][1] = {{mx}, {my}, {mz},}; //magnetômetro
double R[3][3] = {{cos(mx)cos(mz), sin(mx)sin(my)cos(mz)-cos(mx)sin(mz), cos(mx)sin(my)cos(mz)+sin(mx)sin(mz)}, {sin(my)sin(mz), sin(mx)sin(my)sin(mz)+cos(mx)cos(mz), cos(mx)sin(my)sin(mz)-sin(mx)cos(mz)}, {-sin(my), sin(mx)cos(my), cos(mx)cos(my)}}; //Earth-fixed to body frame
double gr[3][1] = {{0}, {0}, {-9.81}}; //vetor gravidade 
double M[12][3] = {{t*t/2, 0, 0}, {0, t*t/2, 0}, {0, 0, t*t/2}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {t, 0, 0}, {0, t, 0}, {0, 0, t}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}}; //matriz acelerômetro
double N[12][3] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {t, 0, 0}, {0, t, 0}, {0, 0, t}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {1, 0, 0}, {0, 1, 0}, {0, 0, 1}}; //matriz giroscópio
            
double X[12][1] = {{x}, {y}, {z}, {r}, {p}, {ya}, {vx}, {vy}, {vz}, {vr}, {vp}, {vya}} = {{0}, {0}, {0}, {0}, {0}, {0}, {0}, {0}, {0}, {0}, {0}, {0}}; //vetor de estados
double A[12][12] = {{1, 0, 0, 0, 0, 0, t, 0, 0, 0, 0, 0}, {0, 1, 0, 0, 0, 0, 0, t, 0, 0, 0, 0}, {0, 0, 1, 0, 0, 0, 0, 0, t, 0, 0, 0}, {0, 0, 0, 1, 0, 0, 0, 0, 0, t, 0, 0,}, {0, 0, 0, 0, 1, 0, 0, 0, 0, 0, t, 0,}, {0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, t}, {0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0}, {0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0}, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0}, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1}}; //matriz de estados
double u[6][1] = {{accx}, {accy}, {accz}, {accr}, {accp}, {accya}}; //vetor de controle
double B[12][6] = {{t*t/2 0 0 0 0 0}, {0 t*t/2 0 0 0 0}, {0 0 t*t/2 0 0 0}, {0 0 0 t*t/2 0 0}, {0 0 0 0 t*t/2 0}, {0 0 0 0 0 t*t/2}, {t 0 0 0 0 0}, {0 t 0 0 0 0}, {0 0 t 0 0 0}, {0 0 0 t 0 0}, {0 0 0 0 t 0}, {0 0 0 0 0 t}}; //matriz de controle
double controle[12][1];

double P[12][12] = {{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, 
                    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, 
                    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, 
                    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, 
                    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, 
                    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, 
                    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, 
                    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, 
                    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, 
                    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, 
                    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, 
                    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}}; //matriz de covariância
double P_[12][12]= {{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, 
                    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, 
                    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, 
                    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, 
                    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, 
                    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, 
                    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, 
                    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, 
                    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, 
                    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, 
                    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, 
                    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}}; //matriz "covariância" para pontos sigma

double lower[12][12]; //para decomposição de Cholesky

double sigma[25][12]; //matriz pontos Sigma

double weight_m[25][1];
double weight_c[25][1];

double media(double _0[12][1], double _1[12][1], double _2[12][1], double _3[12][1], double _4[12][1])
{
            return (_0[i][1] + _1[i][1] + _2[i][1] + _3[i][1] + _4[i][1])/5.0;
}

double covariancia(double media[12][1], double _0[12][1], double _1[12][1], double _2[12][1], double _3[12][1], double _4[12][1])
{
            int i, j, k;
            double media_m[12][5];
            double sum;
            double covar [12][12];
            
            for (i=0, i<12, i++)
            {
                        media_m[i][0] = media[i][1] - _0[i][1];
                        
            }
            
            for (i=0, i<12, i++)
            {
                        media_m[i][1] = media[i][1] - _1[i][1];
                        
            }
            
            for (i=0, i<12, i++)
            {
                        media_m[i][2] = media[i][1] - _2[i][1];
                        
            }
            
            for (i=0, i<12, i++)
            {
                        media_m[i][3] = media[i][1] - _3[i][1];
                        
            }
            
            for (i=0, i<12, i++)
            {
                        media_m[i][4] = media[i][1] - _4[i][1];
                        
            }
            
            for (i=0, i<12, i++)
            {
                        for (j=i, j<12, j++)
                        {
                                    for (k=0, k<5, k++)
                                    {
                                                sum = sum + media_m[i][k]*media_m[j][k];
                                    }
                                    covar[i][j] =  sum/4.0;
                                    covar[j][i] = covar[i][j];
                        }
                        
            }
       
            return covar;
}

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
                // Evaluating L(i, j) using L(j, j)
                for (int k = 0; k < j; k++)
                    sum += (lower[i][k] * lower[j][k]);
                lower[i][j] = (matrix[i][j] - sum) / lower[j][j];
            }
        }
    }
}


int main ()
{
           // X = X*A + B*u;
            a = R*(a-gr);
            X_m1 = X_m0 + M*a + N*g;
            
            //Pontos Sigma
            
            lambda = alfa*alfa*(n+ki)-n;
            P_ = (n+lamba)*P;
            Cholesky_Decomposition(P_);
            
            for (i=0, i<12, i++)
            {
                        sigma[i][0] = mu;
            }
            
            for (i=1, i<13, i++)
            {
                        for (j=0, j<12, j++)
                        {
                                    sigma[j][i] = mu + lower[j][i];
                                    sigma[j][i+12] = mu - lower[j][i];
                        }
            }
            
            //Pesos dos pontos
            
            weight_m[0][0] = lambda/(n+lambda);
            weight_c[0][0] = weight_m[0][0] + (1 + alfa*alfa + beta);  
            
            for (i=1, i<25, i++)
            {
                        weight_m[i][0] = 1/2*(n+lambda);
                        weight_c[i][0] = weight_m[i][0];
            }
}


/*Prediction Step - deve ser integrado à int main()


//f(sigma, u) = deve ser transformado em função
//B*u
for (i=0, i<12, i++)
{
            sum=0;
            for (j=0, j<6, j++)
            {
                        sum = sum + B[i][j]*u[j][0];
            }
            controle[i][0] = sum;
 }
            
//A*sigma
for (i=0, i<12, i++)
{
            for (j=0, j<25, j++)
            {
                        sum = 0;
                        for (k=0, k<12, k++)
                        {
                                    sum = sum + A[i][k]*sigma[k][j];
                        }
                        estado[i][j] = sum; 
             }
 }
 
//A*sigma + B*u
for (i=0, i<12, i++)
{
            for (j=0, j<25, j++)
            {
                        estado[i][j] = estado[i][j] + controle[i][0];
            }
 }

//cálculo média
for (i=0, i<25, i++)
{
            for (j=0, j<12, j++)
            {
                        mu = mu + weight_m[i][0]*estado[j][i];
            }
 }
 
 



/*
roll_quad  = math.atan(2*(q2*q3 + q0*q1), q0^2 - q1^2 - q2^2 - q3^2) -- Roll
pitch_quad = math.asin(-2*(q1*q3 - q0*q2))                           -- Pitch
yaw_quad   = math.atan(2*(q1*q2 + q0*q3), q0^2 + q1^2 - q2^2 - q3^2) -- Yaw
*/
