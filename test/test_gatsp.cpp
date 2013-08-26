#include <fstream>
#include <iostream>
#include <sstream>
#include <vector>

#include "ga/GASimpleGA.h"

#include "gatsp/Problem.h"
#include "gatsp/tsplibParser.h"
#include "gatsp/Waypoint.h"
#include "gatsp/ProblemGenome.h"
#include "gatsp/GeneticAlgorithm.h"

void testWaypoint();
void testProblem();
void testProblemGenome();
void testGatsp();
void runGatsp(unsigned int);
void runNewAlgorithm(unsigned int seed = 0);

int main(int argc, char const *argv[])
{

    // testWaypoint();
    // testProblem();
    //testProblemGenome();
    //testGatsp();
    //runGatsp(10);
    runNewAlgorithm();

    return 0;
}

void testWaypoint()
{
    std::cout << "Making empty waypoint... ";
    gatsp::Waypoint wp_empty;
    std::cout << "Done!" << std::endl;

    std::cout << "Making waypoint at <10, 10>... ";
    gatsp::Waypoint wp_10_10(10, 10);
    std::cout << "Done!" << std::endl;

    std::cout << "Testing operator<< for Waypoint..." << std::endl;
    std::cout << wp_10_10 << std::endl;
    std::cout << "Done!" << std::endl;
}

void testProblem()
{
    std::cout << "Making empty problem... ";
    gatsp::Problem prob_empty;
    std::cout << "Done!" << std::endl;

    std::cout << "Making problem with two waypoints... ";
    gatsp::Waypoint wp1(1,1);
    gatsp::Waypoint wp2(2,2);
    std::vector<gatsp::Waypoint> wp_vec;
    wp_vec.push_back(wp1);
    wp_vec.push_back(wp2);
    gatsp::Problem prob_vec(wp_vec);
    std::cout << "Done!" << std::endl;

    std::cout << "Parsing Berlin52.tsp... ";
    std::ifstream berlin52stream = std::ifstream("berlin52.tsp", std::ios_base::in);
    try
    {
        gatsp::Problem b52 = gatsp::tsplibParser(berlin52stream);
    }
    catch (gatsp::TspParserError e)
    {
        std::cout << e.what();
    }
    std::cout << "Done!" << std::endl;

    std::stringstream tspTextStream;
    tspTextStream << "NAME: karl5" << std::endl;
    tspTextStream << "TYPE: TSP" << std::endl;
    tspTextStream << "COMMENT: 7 locations (mainly) from berlin52" << std::endl;
    tspTextStream << "DIMENSION: 7" << std::endl;
    tspTextStream << "EDGE_WEIGHT_TYPE: EUC_2D" << std::endl;
    tspTextStream << "NODE_COORD_SECTION" << std::endl;
    tspTextStream << "1 565.0 575.0" << std::endl;
    tspTextStream << "2 25.0 185.0" << std::endl;
    tspTextStream << "3 345.0 750.0" << std::endl;
    tspTextStream << "4 945.0 685.0" << std::endl;
    tspTextStream << "5 845.0 655.0" << std::endl;
    tspTextStream << "6 445.0 285.0" << std::endl;
    tspTextStream << "7 25.0 545.0" << std::endl;
    tspTextStream << "EOF" << std::endl;
    std::cout << "Parsing karl5 sstream... ";
    gatsp::Problem karl5;
    try
    {
        karl5 = gatsp::tsplibParser(tspTextStream);
    }
    catch (gatsp::TspParserError e)
    {
        std::cout << e.what() << std::endl;
    }
    std::cout << "Done!" << std::endl;

    std::cout << "Testing stream output from Problem class..." << std::endl;
    std::cout << karl5 << std::endl;
    std::cout << "Done!" << std::endl;

    std::cout << "Seing if karl5's solution is empty" << std::endl;
    auto sol = karl5.solution();
    std::cout << "Elements in solution: " << sol.size() << std::endl;
    std::cout << "Done!" << std::endl;

    std::cout << "Parsing hk48.tsp... ";
    std::ifstream hk48stream = std::ifstream("hk48.tsp", std::ios_base::in);
    try
    {
        gatsp::Problem hk48 = gatsp::tsplibParser(hk48stream);
    }
    catch (gatsp::TspParserError e)
    {
        std::cout << e.what() << std::endl;
    }
    std::cout << "Done!" << std::endl;
}

void testProblemGenome()
{
    std::stringstream tspTextStream;
    tspTextStream << "NAME: line10" << std::endl;
    tspTextStream << "TYPE: TSP" << std::endl;
    tspTextStream << "COMMENT: 10 points on a line" << std::endl;
    tspTextStream << "DIMENSION: 10" << std::endl;
    tspTextStream << "EDGE_WEIGHT_TYPE: EUC_2D" << std::endl;
    tspTextStream << "NODE_COORD_SECTION" << std::endl;
    tspTextStream << "1 0.0 0.0" << std::endl;
    tspTextStream << "2 1.0 0.0" << std::endl;
    tspTextStream << "3 2.0 0.0" << std::endl;
    tspTextStream << "4 3.0 0.0" << std::endl;
    tspTextStream << "5 4.0 0.0" << std::endl;
    tspTextStream << "6 5.0 0.0" << std::endl;
    tspTextStream << "7 6.0 0.0" << std::endl;
    tspTextStream << "8 7.0 0.0" << std::endl;
    tspTextStream << "9 8.0 0.0" << std::endl;
    tspTextStream << "10 9.0 0.0" << std::endl;
    tspTextStream << "EOF" << std::endl;
    std::cout << "Parsing line10 sstream... ";
    gatsp::Problem line10;
    try
    {
        line10 = gatsp::tsplibParser(tspTextStream);
    }
    catch (gatsp::TspParserError e)
    {
        std::cout << e.what() << std::endl;
    }
    std::cout << "Done!" << std::endl;

    std::cout << "Initializing a ProblemGenome..." << std::endl;
    gatsp::ProblemGenome pg(line10);
    gatsp::ProblemGenome::Initialize(pg);
    std::cout << static_cast<GAGenome&>(pg) << std::endl;
    std::cout << "Length: " << pg.score() << std::endl;
    std::cout << "Done!" << std::endl;

    std::cout << "Mutating..." << std::endl;
    pg.mutate(1.0);
    std::cout << static_cast<GAGenome&>(pg) << std::endl;
    std::cout << "Length: " << pg.score() << std::endl;
    std::cout << std::endl;
    std::cout << "Done!" << std::endl;
}

void testGatsp()
{
    std::stringstream tspTextStream;
    tspTextStream << "NAME: line10" << std::endl;
    tspTextStream << "TYPE: TSP" << std::endl;
    tspTextStream << "COMMENT: 10 points on a line" << std::endl;
    tspTextStream << "DIMENSION: 10" << std::endl;
    tspTextStream << "EDGE_WEIGHT_TYPE: EUC_2D" << std::endl;
    tspTextStream << "NODE_COORD_SECTION" << std::endl;
    tspTextStream << "1 0.0 0.0" << std::endl;
    tspTextStream << "2 1.0 0.0" << std::endl;
    tspTextStream << "3 2.0 0.0" << std::endl;
    tspTextStream << "4 3.0 0.0" << std::endl;
    tspTextStream << "5 4.0 0.0" << std::endl;
    tspTextStream << "6 5.0 0.0" << std::endl;
    tspTextStream << "7 6.0 0.0" << std::endl;
    tspTextStream << "8 7.0 0.0" << std::endl;
    tspTextStream << "9 8.0 0.0" << std::endl;
    tspTextStream << "10 9.0 0.0" << std::endl;
    tspTextStream << "EOF" << std::endl;
    std::cout << "Parsing line10 sstream... ";
    gatsp::Problem line10;
    try
    {
        line10 = gatsp::tsplibParser(tspTextStream);
    }
    catch (gatsp::TspParserError e)
    {
        std::cout << e.what() << std::endl;
    }
    std::cout << "Done!" << std::endl;

    gatsp::ProblemGenome pg(line10);
    
    GASimpleGA tspga(pg);
    tspga.minimize();
    tspga.populationSize(100);
    tspga.nGenerations(1000);
    tspga.pMutation(1.0);
    tspga.pCrossover(0.0);
    tspga.selectScores(GAStatistics::AllScores);
    std::cout << "initializing..."; std::cout.flush();
    tspga.initialize(2); // Randomization seed
    std::cout << "evolving..."; std::cout.flush();
    while(!tspga.done()) {
        tspga.step();
        if(tspga.generation() % 10 == 0) {
          std::cout << tspga.generation() << " "; std::cout.flush();
        }
    }

    const GAGenome& best_genome = tspga.statistics().bestIndividual();
    GAGenome& ucn = const_cast<GAGenome&>(best_genome);
    gatsp::ProblemGenome& upg = static_cast<gatsp::ProblemGenome&>(ucn);
    std::cout << "Best Genome: " << typeid(best_genome).name() << std::endl;
    std::cout << "the shortest path found is " << best_genome.score() << "\n";
    std::cout << "this is the distance from the sequence\n";
    std::cout << "Evaluated: " << gatsp::ProblemGenome::Evaluate(upg) << std::endl;
    std::cout << best_genome << "\n\n";
    std::cout << tspga.statistics() << "\n";
}

void runGatsp(unsigned int seed = 0)
{
    std::cout << "Parsing Berlin52.tsp... ";
    gatsp::ProblemGenome my_genome;
    try
    {
        std::ifstream berlin52stream = std::ifstream("berlin52.tsp", std::ios_base::in);
        my_genome = gatsp::tsplibParser(berlin52stream);
    }
    catch (gatsp::TspParserError e)
    {
        std::cout << e.what();
    }
    std::cout << "Done!" << std::endl;

    GASimpleGA tspga(my_genome);
    tspga.minimize();
    tspga.populationSize(100);
    tspga.nGenerations(1000);
    tspga.pMutation(1.0);
    tspga.pCrossover(0.0);
    tspga.selectScores(GAStatistics::AllScores);
    tspga.scoreFilename("Scores.dat");
    tspga.flushFrequency(100);

    std::cout << "initializing..."; std::cout.flush();
    tspga.initialize(seed);

    std::cout << "evolving..."; std::cout.flush();
    while(!tspga.done()) {
        tspga.step();
        if(tspga.generation() % 10 == 0) {
          std::cout << tspga.generation() << " "; std::cout.flush();
        }
    }

    const GAGenome& best_genome = tspga.statistics().bestIndividual();
    std::cout << "the shortest path found is " << best_genome.score() << std::endl;
    std::cout << best_genome << std::endl << std::endl;
    std::cout << tspga.statistics() << std::endl;

    std::cout << "Parsing Berlin52.opt.tour... ";
    try
    {
        std::ifstream berlin52tourstream = std::ifstream("berlin52.opt.tour", std::ios_base::in);
        my_genome.parseTsplibSolution(berlin52tourstream);
    }
    catch (gatsp::TspParserError e)
    {
        std::cout << e.what();
    }
    std::cout << "Done!" << std::endl;
    
    std::cout << "Optimal length is: " << gatsp::ProblemGenome::Evaluate(my_genome) << std::endl;
}

void runNewAlgorithm(unsigned int seed)
{
    std::cout << "Parsing Berlin52.tsp... ";
    gatsp::ProblemGenome my_genome;
    try
    {
        std::ifstream berlin52stream = std::ifstream("berlin52.tsp", std::ios_base::in);
        my_genome = gatsp::tsplibParser(berlin52stream);
    }
    catch (gatsp::TspParserError e)
    {
        std::cout << e.what();
    }
    std::cout << "Done!" << std::endl;

    gatsp::GeneticAlgorithm tspga(my_genome);

    std::cout << "Evolving..."; std::cout.flush();
    tspga.evolve(seed);

    const GAGenome& best_genome = tspga.statistics().bestIndividual();
    std::cout << "the shortest path found is " << best_genome.score() << std::endl;
    std::cout << best_genome << std::endl << std::endl;
    std::cout << tspga.statistics() << std::endl;
}