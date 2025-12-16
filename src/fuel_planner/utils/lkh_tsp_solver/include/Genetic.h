#ifndef _GENETIC_H
#define _GENETIC_H

/*
 * This header specifies the interface for the genetic algorithm part of LKH.
 */

#ifdef __cplusplus
extern "C" {
#endif

#include "GainType.h"

typedef void (*CrossoverFunction)();

#ifndef LKH_EXTERN
#  ifdef LKH_DEFINE_GLOBALS
#    define LKH_EXTERN
#  else
#    define LKH_EXTERN extern
#  endif
#endif

LKH_EXTERN int MaxPopulationSize; /* The maximum size of the population */
LKH_EXTERN int PopulationSize;    /* The current size of the population */

LKH_EXTERN CrossoverFunction Crossover;

LKH_EXTERN int** Population;  /* Array of individuals (solution tours) */
LKH_EXTERN GainType* Fitness; /* The fitness (tour cost) of each individual */

void AddToPopulation(GainType Cost);
void ApplyCrossover(int i, int j);
void FreePopulation();
int HasFitness(GainType Cost);
int LinearSelection(int Size, double Bias);
GainType MergeTourWithIndividual(int i);
void PrintPopulation();
void ReplaceIndividualWithTour(int i, GainType Cost);
int ReplacementIndividual(GainType Cost);

void ERXT();

#ifdef __cplusplus
}  // extern "C"
#endif

#endif
