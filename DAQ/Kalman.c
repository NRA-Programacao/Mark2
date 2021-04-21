#include <stdio.h>
#include <math.h>
#include <bits/stdc++.h>

using namespace std;

double t; //intervalo de tempo do filtro de Kalman
double alfa = 1e-3, ki = 0, lambda = 0;
int beta = 2, n = 12, i, j;
double mu[12][1] = {{0}, {0}, {0}, {0}, {0}, {0}, {0}, {0}, {0}, {0}, {0}, {0}}; //vetor média de cada estado
double mu_0[12][1] ={{0}, {0}, {0}, {0}, {0}, {0}, {0}, {0}, {0}, {0}, {0}, {0}}; //vetor média de cada estado no passo anterior

double X_m0[12][1] = {{0}, {0}, {0}, {0}, {0}, {0}, {0}, {0}, {0}, {0}, {0}, {0}}; //estados medidos k
double X_m1[12][1] = {{0}, {0}, {0}, {0}, {0}, {0}, {0}, {0}, {0}, {0}, {0}, {0}}; //estados medidos k-1
double X_m2[12][1] = {{0}, {0}, {0}, {0}, {0}, {0}, {0}, {0}, {0}, {0}, {0}, {0}}; //estados medidos k-2
double X_m3[12][1] = {{0}, {0}, {0}, {0}, {0}, {0}, {0}, {0}, {0}, {0}, {0}, {0}}; //estados medidos k-3
double X_m4[12][1] = {{0}, {0}, {0}, {0}, {0}, {0}, {0}, {0}, {0}, {0}, {0}, {0}}; //estados medidos k-4

double a[3][1] = {{ax}, {ay}, {az}};  //acelerômetro
double g[3][1] = {{gx}, {gy}, {gz}};  //giroscópio
double m[3][1] = {{mx}, {my}, {mz},}; //magnetômetro

double R[3][3] = {{cos(mx)cos(mz), sin(mx)sin(my)cos(mz)-cos(mx)sin(mz), cos(mx)sin(my)cos(mz)+sin(mx)sin(mz)}, //Earth-fixed to body frame
                  {sin(my)sin(mz), sin(mx)sin(my)sin(mz)+cos(mx)cos(mz), cos(mx)sin(my)sin(mz)-sin(mx)cos(mz)}, 
                  {-sin(my), sin(mx)cos(my), cos(mx)cos(my)}}; 

double gr[3][1] = {{0}, {0}, {-9.81}}; //vetor gravidade 

double M[12][3] = {{t*t/2, 0, 0}, //matriz acelerômetro
                   {0, t*t/2, 0}, 
                   {0, 0, t*t/2}, 
                   {0, 0, 0}, 
                   {0, 0, 0}, 
                   {0, 0, 0},
                   {t, 0, 0},
                   {0, t, 0},
                   {0, 0, t}, 
                   {0, 0, 0}, 
                   {0, 0, 0}, 
                   {0, 0, 0}}; 

double N[12][3] = {{0, 0, 0}, //matriz giroscópio
                   {0, 0, 0}, 
                   {0, 0, 0},
                   {t, 0, 0},
                   {0, t, 0}, 
                   {0, 0, t}, 
                   {0, 0, 0},
                   {0, 0, 0}, 
                   {0, 0, 0},
                   {1, 0, 0},
                   {0, 1, 0},
                   {0, 0, 1}}; 
            
double X[12][1] = {{x}, {y}, {z}, {r}, {p}, {ya}, {vx}, {vy}, {vz}, {vr}, {vp}, {vya}} = {{0}, {0}, {0}, {0}, {0}, {0}, {0}, {0}, {0}, {0}, {0}, {0}}; //vetor de estados

double A[12][12] = {{1, 0, 0, 0, 0, 0, t, 0, 0, 0, 0, 0}, //matriz de estados
                    {0, 1, 0, 0, 0, 0, 0, t, 0, 0, 0, 0}, 
                    {0, 0, 1, 0, 0, 0, 0, 0, t, 0, 0, 0},
                    {0, 0, 0, 1, 0, 0, 0, 0, 0, t, 0, 0}, 
                    {0, 0, 0, 0, 1, 0, 0, 0, 0, 0, t, 0},
                    {0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, t}, 
                    {0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0}, 
                    {0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0}, 
                    {0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0}, 
                    {0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0}, 
                    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0}, 
                    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1}};

double u[6][1] = {{accx}, {accy}, {accz}, {accr}, {accp}, {accya}}; //vetor de controle

double B[12][6] = {{t*t/2 0 0 0 0 0}, //matriz de controle
                   {0 t*t/2 0 0 0 0}, 
                   {0 0 t*t/2 0 0 0}, 
                   {0 0 0 t*t/2 0 0}, 
                   {0 0 0 0 t*t/2 0}, 
                   {0 0 0 0 0 t*t/2}, 
                   {t 0 0 0 0 0}, 
                   {0 t 0 0 0 0}, 
                   {0 0 t 0 0 0}, 
                   {0 0 0 t 0 0},
                   {0 0 0 0 t 0}, 
                   {0 0 0 0 0 t}};

double controle[12][1];

double P[12][12] = {{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, //matriz de covariância
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
                    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}}; 

double P_[12][12]= {{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, //matriz "covariância" para pontos sigma
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
                    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}}; 

double lower[12][12]; //para decomposição de Cholesky

double sigma[25][12]; //matriz pontos Sigma

double weight_m[25][1]; //vetor de pesos dos pontos sigma para média
double weight_c[25][1]; //vetor de pesos de pontos sigma para covariância

/*calcula a média de 5 vetores de estado (objetivo é usar os últimos 5)
double media(double _0[12][1], double _1[12][1], double _2[12][1], double _3[12][1], double _4[12][1])
{
            return (_0[i][1] + _1[i][1] + _2[i][1] + _3[i][1] + _4[i][1])/5.0;
}

//calcula a covariancia entre 5 vetores de estado (objetivo é usar os últimos 5)
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
*/
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

//determinação dos pontos sigma
void sigma_points (void)
{
            lambda = alfa*alfa*(n+ki)-n;
            P_ = (n+lamba)*P;
            Cholesky_Decomposition(P_);
            
            mu_0 = mu; //atribuição para não perder as médias do passo anterior
            for (i=0, i<12, i++) //a primeira coluna da matriz sigma é a coluna de médias anteriores
            {
                        sigma[i][0] = mu_0;
            }
            
            for (i=1, i<13, i++) //determinação dos pontos sigma restantes
            {
                        for (j=0, j<12, j++)
                        {
                                    sigma[j][i] = mu_0 + lower[j][i];
                                    sigma[j][i+12] = mu_0 - lower[j][i];
                        }
            }
            
            //Pesos dos pontos sigma
            
            weight_m[0][0] = lambda/(n+lambda);
            weight_c[0][0] = weight_m[0][0] + (1 + alfa*alfa + beta);  
            
            for (i=1, i<25, i++)
            {
                        weight_m[i][0] = 1/2*(n+lambda);
                        weight_c[i][0] = weight_m[i][0];
            }
}

int main ()
{
           // X = X*A + B*u;
            a = R*(a-gr);
          
            
            
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
            g[i][0] = sum; //vetor g ainda não foi declarado
 }
            
//A*sigma[coluna]
for (i=0, i<25, i++)
{
            for (j=0, j<12, j++)
            {
                        sum = 0;
                        for (k=0, k<12, k++)
                        {
                                    sum = sum + A[k][j]*sigma[k][i];
                        }
                        f[j][i] = sum; //a matriz f ainda não foi declarada
             }
 }
 
//f[coluna] + B*u
for (i=0, i<25, i++)
{
            for (j<0, j<12, j++)
            {
                        f[j][i] = f[j][i] + g[i][0];
            }
}

//peso*(f[coluna]-mu)(transposta(f[coluna]-mu)
for (i=0, i<25, i++)
{
            for (j=0, j<12, j++)
            {
                        X_m0[j][i] = f[j][i] - mu[j][0]; //X_m0 é apenas matriz auxiliar [12][25], a declaração está errada
            }
            for (j=0, j<12, j++)
            {
                        sum = 0;
                        for (k=0, k<12, k++)
                        {
                                   sum = sum + X_m0[j][i]*X_m0[i][k];
                        }
                        X_m1[i][j] = weight_c[i][0]*sum; //X_m1 é apenas matriz auxiliar [12][12], a declaração está errada          
}

//soma das colunas X_m1
for (i=0, i<25, i++)
{
            for (j=0, j<12, j++)
            {
                        X_m2[j][0] = X_m2[j][0] + X_m1[j][i]; //X_m1 vetor de covariância [12][1], a declaração está errada, ainda é necessário implementar a soma com Q
            }
}

//cálculo média
mu_0 = mu;
for (i=0, i<25, i++)
{
            for (j=0, j<12, j++)
            {
                        mu = mu + weight_m[i][0]*estado[j][i];
            }
            
 }
 
 //Update step - deve ser integrado à int main
 //Modelo de observação
 //M*a
 for (i=0, i<25, i++)
 {
            for (j=0, j<12, j++)
            {
                        sum = 0;
                        for (k=0, k<3, k++)
                        {
                                    sum = sum + M[j][k]*a[k][0];
                        }
                        X_m3[j][i] = sum; //X_m3 é apenas matriz auxiliar [12][25], a declaração está errada
            }
            
 }
 
 //N*g
 for (i=0, i<25, i++)
 {
            for (j=0, j<12, j++)
            {
                        sum = 0;
                        for (k=0, k<3, k++)
                        {
                                    sum = sum + N[j][k]*g[k][0];
                        }
                        X_m4[j][i] = X_m3 + sum; //X_m4 é apenas matriz auxiliar [12][25], a declaração está errada
            }
            
 }
 
 //cálculo da soma das colunas de X_m4, multiplicadas por seus respectivos pesos
 for (i=0, i<12, i++)
 {
            for (j=0, j<25, j++)
            {
                        X_m5[i][0] = X_m4[i][j]*weight_m[j][0]; //X_m5 é vetor auxiliar [12][1], a declaração está errada
            }
 }
 
 //peso*(X_m4[coluna]-X_m5)(transposta(X_m4[coluna]-X_m5)
for (i=0, i<25, i++)
{
            for (j=0, j<12, j++)
            {
                        X_m6[j][i] = X_m4[j][i] - X_m5[j][0]; //X_m6 é apenas matriz auxiliar [12][25], ainda não declarado
            }
            for (j=0, j<12, j++)
            {
                        sum = 0;
                        for (k=0, k<12, k++)
                        {
                                   sum = sum + X_m6[j][i]*X_m6[i][k];
                        }
                        X_m7[i][j] = weight_c[i][0]*sum; //X_m7 é apenas matriz auxiliar [12][12], ainda não declarado         
}

//soma das colunas X_m7
for (i=0, i<25, i++)
{
            for (j=0, j<12, j++)
            {
                        X_m8[j][0] = X_m8[j][0] + X_m7[j][i]; //X_m8 vetor S_t [12][1], ainda não declarado, ainda é necessário implementar a soma com R_t
            }
}

//matriz de correlação cruzada
//sigma-mu e Z-mu
for (i=0, i<25, i++)
{
            for (j=0, j<12, j++)
            {
                        X_m9[j][0] = sigma[j][i] - mu[j][0];  //X_m9 e X_m10 são vetores auxiliares [12][1] ainda não declarados
                        X_m10[j][0] = X_m4[j][i] - X_m8[j][0];
            }
} 

//(sigma-mu)*transposta(Z-mu)





for (i=0, i<25, i++)
{
            for (j=0, j<12, j++)
            {
                        X_m9[j][0] = sigma[j][i] - mu[j][0];  //X_m9 e X_m10 são vetores auxiliares [12][1] ainda não declarados
                        X_m10[j][0] = X_m4[j][i] - X_m8[j][0];
            }
            for (j=0, j<12, j++)
            {
                        sum = 0;
                        for (k=0, k<12, k++)
                        {
                                   sum = sum + X_m9[j][i]*X_m10[i][k];
                        }
                        X_m11[i][j] = weight_c[i][0]*sum; //X_m11 é apenas matriz auxiliar [12][12], a declaração está errada          
}

//soma das colunas X_m11
for (i=0, i<25, i++)
{
            for (j=0, j<12, j++)
            {
                        X_m12[j][0] = X_m12[j][0] + X_m11[j][i]; //X_m12 vetor SIGMA_BARRA_txz [12][1], ainda não declarado
            }
}

/*
roll_quad  = math.atan(2*(q2*q3 + q0*q1), q0^2 - q1^2 - q2^2 - q3^2) -- Roll
pitch_quad = math.asin(-2*(q1*q3 - q0*q2))                           -- Pitch
yaw_quad   = math.atan(2*(q1*q2 + q0*q3), q0^2 + q1^2 - q2^2 - q3^2) -- Yaw
*/
