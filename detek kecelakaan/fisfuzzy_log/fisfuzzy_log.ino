//***********************************************************************
// Matlab .fis to arduino C converter v2.0.1.25122016                   
// - Karthik Nadig, USA                                                  
// Please report bugs to:                                                
// https://github.com/karthiknadig/ArduinoFIS/issues                     
// If you don't have a GitHub account mail to karthiknadig@gmail.com     
//***********************************************************************

#include "fis_header.h"

// Number of inputs to the fuzzy inference system
const int fis_gcI = 9;
// Number of outputs to the fuzzy inference system
const int fis_gcO = 1;
// Number of rules to the fuzzy inference system
const int fis_gcR = 7;

FIS_TYPE g_fisInput[fis_gcI];
FIS_TYPE g_fisOutput[fis_gcO];

// Setup routine runs once when you press reset:
void setup()
{
    // initialize the Analog pins for input.
    // Pin mode for Input: accx
    pinMode(0 , INPUT);
    // Pin mode for Input: accy
    pinMode(1 , INPUT);
    // Pin mode for Input: accz
    pinMode(2 , INPUT);
    // Pin mode for Input: gyrox
    pinMode(3 , INPUT);
    // Pin mode for Input: gyroy
    pinMode(4 , INPUT);
    // Pin mode for Input: gyroz
    pinMode(5 , INPUT);
    // Pin mode for Input: tumbukanx
    pinMode(6 , INPUT);
    // Pin mode for Input: tumbukany
    pinMode(7 , INPUT);
    // Pin mode for Input: tumbukanz
    pinMode(8 , INPUT);


    // initialize the Analog pins for output.
    // Pin mode for Output: indikasi
    pinMode(9 , OUTPUT);

}

// Loop routine runs over and over again forever:
void loop()
{
    // Read Input: accx
    g_fisInput[0] = analogRead(0);
    // Read Input: accy
    g_fisInput[1] = analogRead(1);
    // Read Input: accz
    g_fisInput[2] = analogRead(2);
    // Read Input: gyrox
    g_fisInput[3] = analogRead(3);
    // Read Input: gyroy
    g_fisInput[4] = analogRead(4);
    // Read Input: gyroz
    g_fisInput[5] = analogRead(5);
    // Read Input: tumbukanx
    g_fisInput[6] = analogRead(6);
    // Read Input: tumbukany
    g_fisInput[7] = analogRead(7);
    // Read Input: tumbukanz
    g_fisInput[8] = analogRead(8);

    g_fisOutput[0] = 0;

    fis_evaluate();

    // Set output vlaue: indikasi
    analogWrite(9 , g_fisOutput[0]);

}

//***********************************************************************
// Support functions for Fuzzy Inference System                          
//***********************************************************************
// Triangular Member Function
FIS_TYPE fis_trimf(FIS_TYPE x, FIS_TYPE* p)
{
    FIS_TYPE a = p[0], b = p[1], c = p[2];
    FIS_TYPE t1 = (x - a) / (b - a);
    FIS_TYPE t2 = (c - x) / (c - b);
    if ((a == b) && (b == c)) return (FIS_TYPE) (x == a);
    if (a == b) return (FIS_TYPE) (t2*(b <= x)*(x <= c));
    if (b == c) return (FIS_TYPE) (t1*(a <= x)*(x <= b));
    t1 = min(t1, t2);
    return (FIS_TYPE) max(t1, 0);
}

// Gaussian Member Function
FIS_TYPE fis_gaussmf(FIS_TYPE x, FIS_TYPE* p)
{
    FIS_TYPE s = p[0], c = p[1];
    FIS_TYPE t = (x - c) / s;
    return exp(-(t * t) / 2);
}

// Product of Sigmoid Member Function
FIS_TYPE fis_psigmf(FIS_TYPE x, FIS_TYPE* p)
{
    return (fis_sigmf(x, p) * fis_sigmf(x, p + 2));
}

// Sigmoid Member Function
FIS_TYPE fis_sigmf(FIS_TYPE x, FIS_TYPE* p)
{
    FIS_TYPE a = p[0], c = p[1];
    return (FIS_TYPE) (1.0 / (1.0 + exp(-a *(x - c))));
}

// Generalized Bell Member Function
FIS_TYPE fis_gbellmf(FIS_TYPE x, FIS_TYPE* p)
{
    FIS_TYPE a = p[0], b = p[1], c = p[2];
    FIS_TYPE t = (x - c) / a;
    if ((t == 0) && (b == 0)) return (FIS_TYPE) 0.5;
    if ((t == 0) && (b < 0)) return (FIS_TYPE) 0;
    return (FIS_TYPE) (1.0 / (1.0 + pow(t, b)));
}

FIS_TYPE fis_min(FIS_TYPE a, FIS_TYPE b)
{
    return min(a, b);
}

FIS_TYPE fis_max(FIS_TYPE a, FIS_TYPE b)
{
    return max(a, b);
}

FIS_TYPE fis_array_operation(FIS_TYPE *array, int size, _FIS_ARR_OP pfnOp)
{
    int i;
    FIS_TYPE ret = 0;

    if (size == 0) return ret;
    if (size == 1) return array[0];

    ret = array[0];
    for (i = 1; i < size; i++)
    {
        ret = (*pfnOp)(ret, array[i]);
    }

    return ret;
}


//***********************************************************************
// Data for Fuzzy Inference System                                       
//***********************************************************************
// Pointers to the implementations of member functions
_FIS_MF fis_gMF[] =
{
    fis_trimf, fis_gaussmf, fis_psigmf, fis_sigmf, fis_gbellmf
};

// Count of member function for each Input
int fis_gIMFCount[] = { 3, 3, 3, 3, 3, 3, 3, 3, 3 };

// Count of member function for each Output 
int fis_gOMFCount[] = { 3 };

// Coefficients for the Input Member Functions
FIS_TYPE fis_gMFI0Coeff1[] = { -0.4, 0, 0.4 };
FIS_TYPE fis_gMFI0Coeff2[] = { 0.1, 0.5, 0.9 };
FIS_TYPE fis_gMFI0Coeff3[] = { 0.6, 1, 1.4 };
FIS_TYPE* fis_gMFI0Coeff[] = { fis_gMFI0Coeff1, fis_gMFI0Coeff2, fis_gMFI0Coeff3 };
FIS_TYPE fis_gMFI1Coeff1[] = { -0.4, 0, 0.4 };
FIS_TYPE fis_gMFI1Coeff2[] = { 0.1, 0.5, 0.9 };
FIS_TYPE fis_gMFI1Coeff3[] = { 0.6, 1, 1.4 };
FIS_TYPE* fis_gMFI1Coeff[] = { fis_gMFI1Coeff1, fis_gMFI1Coeff2, fis_gMFI1Coeff3 };
FIS_TYPE fis_gMFI2Coeff1[] = { -0.4, 0, 0.4 };
FIS_TYPE fis_gMFI2Coeff2[] = { 0.1, 0.5, 0.9 };
FIS_TYPE fis_gMFI2Coeff3[] = { 0.6, 1, 1.4 };
FIS_TYPE* fis_gMFI2Coeff[] = { fis_gMFI2Coeff1, fis_gMFI2Coeff2, fis_gMFI2Coeff3 };
FIS_TYPE fis_gMFI3Coeff1[] = { -0.4, 0, 0.4 };
FIS_TYPE fis_gMFI3Coeff2[] = { 0.1, 0.5, 0.9 };
FIS_TYPE fis_gMFI3Coeff3[] = { 0.6, 1, 1.4 };
FIS_TYPE* fis_gMFI3Coeff[] = { fis_gMFI3Coeff1, fis_gMFI3Coeff2, fis_gMFI3Coeff3 };
FIS_TYPE fis_gMFI4Coeff1[] = { -0.4, 0, 0.4 };
FIS_TYPE fis_gMFI4Coeff2[] = { 0.1, 0.5, 0.9 };
FIS_TYPE fis_gMFI4Coeff3[] = { 0.6, 1, 1.4 };
FIS_TYPE* fis_gMFI4Coeff[] = { fis_gMFI4Coeff1, fis_gMFI4Coeff2, fis_gMFI4Coeff3 };
FIS_TYPE fis_gMFI5Coeff1[] = { -0.4, 0, 0.4 };
FIS_TYPE fis_gMFI5Coeff2[] = { 0.1, 0.5, 0.9 };
FIS_TYPE fis_gMFI5Coeff3[] = { 0.6, 1, 1.4 };
FIS_TYPE* fis_gMFI5Coeff[] = { fis_gMFI5Coeff1, fis_gMFI5Coeff2, fis_gMFI5Coeff3 };
FIS_TYPE fis_gMFI6Coeff1[] = { -0.4, 0, 0.4 };
FIS_TYPE fis_gMFI6Coeff2[] = { 0.1, 0.5, 0.9 };
FIS_TYPE fis_gMFI6Coeff3[] = { 0.6, 1, 1.4 };
FIS_TYPE* fis_gMFI6Coeff[] = { fis_gMFI6Coeff1, fis_gMFI6Coeff2, fis_gMFI6Coeff3 };
FIS_TYPE fis_gMFI7Coeff1[] = { -0.4, 0, 0.4 };
FIS_TYPE fis_gMFI7Coeff2[] = { 0.1, 0.5, 0.9 };
FIS_TYPE fis_gMFI7Coeff3[] = { 0.6, 1, 1.4 };
FIS_TYPE* fis_gMFI7Coeff[] = { fis_gMFI7Coeff1, fis_gMFI7Coeff2, fis_gMFI7Coeff3 };
FIS_TYPE fis_gMFI8Coeff1[] = { -0.4, 0, 0.4 };
FIS_TYPE fis_gMFI8Coeff2[] = { 0.1, 0.5, 0.9 };
FIS_TYPE fis_gMFI8Coeff3[] = { 0.6, 1, 1.4 };
FIS_TYPE* fis_gMFI8Coeff[] = { fis_gMFI8Coeff1, fis_gMFI8Coeff2, fis_gMFI8Coeff3 };
FIS_TYPE** fis_gMFICoeff[] = { fis_gMFI0Coeff, fis_gMFI1Coeff, fis_gMFI2Coeff, fis_gMFI3Coeff, fis_gMFI4Coeff, fis_gMFI5Coeff, fis_gMFI6Coeff, fis_gMFI7Coeff, fis_gMFI8Coeff };

// Coefficients for the Output Member Functions
FIS_TYPE fis_gMFO0Coeff1[] = { 0.1699, 6.939e-18 };
FIS_TYPE fis_gMFO0Coeff2[] = { 13.73, 0.3, -6.865, 0.9401 };
FIS_TYPE fis_gMFO0Coeff3[] = { 0.2, 2.5, 1 };
FIS_TYPE* fis_gMFO0Coeff[] = { fis_gMFO0Coeff1, fis_gMFO0Coeff2, fis_gMFO0Coeff3 };
FIS_TYPE** fis_gMFOCoeff[] = { fis_gMFO0Coeff };

// Input membership function set
int fis_gMFI0[] = { 0, 0, 0 };
int fis_gMFI1[] = { 0, 0, 0 };
int fis_gMFI2[] = { 0, 0, 0 };
int fis_gMFI3[] = { 0, 0, 0 };
int fis_gMFI4[] = { 0, 0, 0 };
int fis_gMFI5[] = { 0, 0, 0 };
int fis_gMFI6[] = { 0, 0, 0 };
int fis_gMFI7[] = { 0, 0, 0 };
int fis_gMFI8[] = { 0, 0, 0 };
int* fis_gMFI[] = { fis_gMFI0, fis_gMFI1, fis_gMFI2, fis_gMFI3, fis_gMFI4, fis_gMFI5, fis_gMFI6, fis_gMFI7, fis_gMFI8};

// Output membership function set
int fis_gMFO0[] = { 1, 2, 4 };
int* fis_gMFO[] = { fis_gMFO0};

// Rule Weights
FIS_TYPE fis_gRWeight[] = { 1, 1, 1, 1, 1, 1, 1 };

// Rule Type
int fis_gRType[] = { 1, 1, 1, 1, 1, 1, 1 };

// Rule Inputs
int fis_gRI0[] = { 1, 1, 1, 1, 1, 1, 1, 1, 1 };
int fis_gRI1[] = { 3, 3, 3, 1, 1, 1, 3, 3, 3 };
int fis_gRI2[] = { 2, 2, 2, 1, 1, 3, 2, 2, 2 };
int fis_gRI3[] = { 3, 1, 2, 3, 3, 1, 1, 2, 3 };
int fis_gRI4[] = { 0, 0, 0, 1, 3, 3, 3, 3, 1 };
int fis_gRI5[] = { 0, 2, 1, 3, 1, 2, 3, 3, 1 };
int fis_gRI6[] = { 0, 2, 1, 3, 0, 0, 0, 3, 1 };
int* fis_gRI[] = { fis_gRI0, fis_gRI1, fis_gRI2, fis_gRI3, fis_gRI4, fis_gRI5, fis_gRI6 };

// Rule Outputs
int fis_gRO0[] = { 1 };
int fis_gRO1[] = { 3 };
int fis_gRO2[] = { 2 };
int fis_gRO3[] = { 2 };
int fis_gRO4[] = { 3 };
int fis_gRO5[] = { 3 };
int fis_gRO6[] = { 1 };
int* fis_gRO[] = { fis_gRO0, fis_gRO1, fis_gRO2, fis_gRO3, fis_gRO4, fis_gRO5, fis_gRO6 };

// Input range Min
FIS_TYPE fis_gIMin[] = { 0, 0, 0, 0, 0, 0, 0, 0, 0 };

// Input range Max
FIS_TYPE fis_gIMax[] = { 1, 1, 1, 1, 1, 1, 1, 1, 1 };

// Output range Min
FIS_TYPE fis_gOMin[] = { 0 };

// Output range Max
FIS_TYPE fis_gOMax[] = { 1 };

//***********************************************************************
// Data dependent support functions for Fuzzy Inference System           
//***********************************************************************
FIS_TYPE fis_MF_out(FIS_TYPE** fuzzyRuleSet, FIS_TYPE x, int o)
{
    FIS_TYPE mfOut;
    int r;

    for (r = 0; r < fis_gcR; ++r)
    {
        int index = fis_gRO[r][o];
        if (index > 0)
        {
            index = index - 1;
            mfOut = (fis_gMF[fis_gMFO[o][index]])(x, fis_gMFOCoeff[o][index]);
        }
        else if (index < 0)
        {
            index = -index - 1;
            mfOut = 1 - (fis_gMF[fis_gMFO[o][index]])(x, fis_gMFOCoeff[o][index]);
        }
        else
        {
            mfOut = 0;
        }

        fuzzyRuleSet[0][r] = fis_min(mfOut, fuzzyRuleSet[1][r]);
    }
    return fis_array_operation(fuzzyRuleSet[0], fis_gcR, fis_max);
}

FIS_TYPE fis_defuzz_centroid(FIS_TYPE** fuzzyRuleSet, int o)
{
    FIS_TYPE step = (fis_gOMax[o] - fis_gOMin[o]) / (FIS_RESOLUSION - 1);
    FIS_TYPE area = 0;
    FIS_TYPE momentum = 0;
    FIS_TYPE dist, slice;
    int i;

    // calculate the area under the curve formed by the MF outputs
    for (i = 0; i < FIS_RESOLUSION; ++i){
        dist = fis_gOMin[o] + (step * i);
        slice = step * fis_MF_out(fuzzyRuleSet, dist, o);
        area += slice;
        momentum += slice*dist;
    }

    return ((area == 0) ? ((fis_gOMax[o] + fis_gOMin[o]) / 2) : (momentum / area));
}

//***********************************************************************
// Fuzzy Inference System                                                
//***********************************************************************
void fis_evaluate()
{
    FIS_TYPE fuzzyInput0[] = { 0, 0, 0 };
    FIS_TYPE fuzzyInput1[] = { 0, 0, 0 };
    FIS_TYPE fuzzyInput2[] = { 0, 0, 0 };
    FIS_TYPE fuzzyInput3[] = { 0, 0, 0 };
    FIS_TYPE fuzzyInput4[] = { 0, 0, 0 };
    FIS_TYPE fuzzyInput5[] = { 0, 0, 0 };
    FIS_TYPE fuzzyInput6[] = { 0, 0, 0 };
    FIS_TYPE fuzzyInput7[] = { 0, 0, 0 };
    FIS_TYPE fuzzyInput8[] = { 0, 0, 0 };
    FIS_TYPE* fuzzyInput[fis_gcI] = { fuzzyInput0, fuzzyInput1, fuzzyInput2, fuzzyInput3, fuzzyInput4, fuzzyInput5, fuzzyInput6, fuzzyInput7, fuzzyInput8, };
    FIS_TYPE fuzzyOutput0[] = { 0, 0, 0 };
    FIS_TYPE* fuzzyOutput[fis_gcO] = { fuzzyOutput0, };
    FIS_TYPE fuzzyRules[fis_gcR] = { 0 };
    FIS_TYPE fuzzyFires[fis_gcR] = { 0 };
    FIS_TYPE* fuzzyRuleSet[] = { fuzzyRules, fuzzyFires };
    FIS_TYPE sW = 0;

    // Transforming input to fuzzy Input
    int i, j, r, o;
    for (i = 0; i < fis_gcI; ++i)
    {
        for (j = 0; j < fis_gIMFCount[i]; ++j)
        {
            fuzzyInput[i][j] =
                (fis_gMF[fis_gMFI[i][j]])(g_fisInput[i], fis_gMFICoeff[i][j]);
        }
    }

    int index = 0;
    for (r = 0; r < fis_gcR; ++r)
    {
        if (fis_gRType[r] == 1)
        {
            fuzzyFires[r] = FIS_MAX;
            for (i = 0; i < fis_gcI; ++i)
            {
                index = fis_gRI[r][i];
                if (index > 0)
                    fuzzyFires[r] = fis_min(fuzzyFires[r], fuzzyInput[i][index - 1]);
                else if (index < 0)
                    fuzzyFires[r] = fis_min(fuzzyFires[r], 1 - fuzzyInput[i][-index - 1]);
                else
                    fuzzyFires[r] = fis_min(fuzzyFires[r], 1);
            }
        }
        else
        {
            fuzzyFires[r] = FIS_MIN;
            for (i = 0; i < fis_gcI; ++i)
            {
                index = fis_gRI[r][i];
                if (index > 0)
                    fuzzyFires[r] = fis_max(fuzzyFires[r], fuzzyInput[i][index - 1]);
                else if (index < 0)
                    fuzzyFires[r] = fis_max(fuzzyFires[r], 1 - fuzzyInput[i][-index - 1]);
                else
                    fuzzyFires[r] = fis_max(fuzzyFires[r], 0);
            }
        }

        fuzzyFires[r] = fis_gRWeight[r] * fuzzyFires[r];
        sW += fuzzyFires[r];
    }

    if (sW == 0)
    {
        for (o = 0; o < fis_gcO; ++o)
        {
            g_fisOutput[o] = ((fis_gOMax[o] + fis_gOMin[o]) / 2);
        }
    }
    else
    {
        for (o = 0; o < fis_gcO; ++o)
        {
            g_fisOutput[o] = fis_defuzz_centroid(fuzzyRuleSet, o);
        }
    }
}
