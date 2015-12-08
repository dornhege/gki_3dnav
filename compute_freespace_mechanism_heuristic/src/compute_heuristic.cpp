#include <sbpl/discrete_space_information/environment_navxythetalat.h>
#include <sbpl/utils/key.h>
#include <string>
#include <string.h>
#include <getopt.h>
#include <stdlib.h>
#include <queue>
#include <fstream>
#include <sstream>
#include <iomanip>

/**
 * TODO read in mprims
 * Check what environ does with these for precomputations
 * setup grid with res + cells size (param)
 * for each theta
 *  compute dijkstra style (generalized) heuristic
 *  until 90%?
 * write/safe heuristic
 * display live?
 */

static int g_maxCost = INFINITECOST;


// This is basically just to get to the motion primitives functionalities
class EnvironmentMotionPrims : public EnvironmentNAVXYTHETALATTICE 
{
    public:
    // need actions

    EnvironmentMotionPrims(int width, int height,
            double nominalvel_mpersecs, double timetoturn45degsinplace_secs) {
        EnvNAVXYTHETALATCfg.EnvWidth_c = width;
        EnvNAVXYTHETALATCfg.EnvHeight_c = height;
        EnvNAVXYTHETALATCfg.nominalvel_mpersecs = nominalvel_mpersecs;
        EnvNAVXYTHETALATCfg.timetoturn45degsinplace_secs = timetoturn45degsinplace_secs;
    }

    const EnvNAVXYTHETALATConfig_t getConfig() const {
        return EnvNAVXYTHETALATCfg;
    }

    bool readPrimitives(const std::string & fname) {
        FILE* fMotPrims = fopen(fname.c_str(), "r");
        if(!fMotPrims)
            return false;

        // copied from EnvironmentNAVXYTHETALATTICE, but instead of checking if values match the EnvNAVXYTHETALATCfg
        // it sets these from the mprims file
        char sTemp[1024], sExpected[1024];
        float fTemp;
        int dTemp;
        int totalNumofActions = 0;

        SBPL_PRINTF("Reading in motion primitives...");

        //read in the resolution
        strcpy(sExpected, "resolution_m:");
        if (fscanf(fMotPrims, "%s", sTemp) == 0) return false;
        if (strcmp(sTemp, sExpected) != 0) {
            SBPL_ERROR("ERROR: expected %s but got %s\n", sExpected, sTemp);
            fclose(fMotPrims);
            return false;
        }
        if (fscanf(fMotPrims, "%f", &fTemp) == 0) return false;
        EnvNAVXYTHETALATCfg.cellsize_m = fTemp;

        //read in the angular resolution
        strcpy(sExpected, "numberofangles:");
        if (fscanf(fMotPrims, "%s", sTemp) == 0) return false;
        if (strcmp(sTemp, sExpected) != 0) {
            SBPL_ERROR("ERROR: expected %s but got %s\n", sExpected, sTemp);
            fclose(fMotPrims);
            return false;
        }
        if (fscanf(fMotPrims, "%d", &dTemp) == 0) return false;
        EnvNAVXYTHETALATCfg.NumThetaDirs = dTemp;

        //read in the total number of actions
        strcpy(sExpected, "totalnumberofprimitives:");
        if (fscanf(fMotPrims, "%s", sTemp) == 0) return false;
        if (strcmp(sTemp, sExpected) != 0) {
            SBPL_ERROR("ERROR: expected %s but got %s\n", sExpected, sTemp);
            fclose(fMotPrims);
            return false;
        }
        if (fscanf(fMotPrims, "%d", &totalNumofActions) == 0) {
            fclose(fMotPrims);
            return false;
        }

        for (int i = 0; i < totalNumofActions; i++) {
            SBPL_xytheta_mprimitive motprim;

            if (EnvironmentNAVXYTHETALATTICE::ReadinMotionPrimitive(&motprim, fMotPrims) == false) return false;

            EnvNAVXYTHETALATCfg.mprimV.push_back(motprim);
        }
        SBPL_PRINTF("done ");

        fclose(fMotPrims);

        // manually trigger here to get computed actions
        InitializeEnvConfig(&EnvNAVXYTHETALATCfg.mprimV);

        return true;
    }

    virtual int SizeofCreatedEnv(){ }
    virtual void PrintState(int, bool, FILE*) { }
    virtual int GetFromToHeuristic(int FromStateID, int ToStateID) { }
    virtual int GetGoalHeuristic(int stateID) { }
    virtual int GetStartHeuristic(int stateID) { }
    virtual void SetAllActionsandAllOutcomes(CMDPSTATE* state) { }
    virtual void GetPreds(int TargetStateID, std::vector<int>* PredIDV, std::vector<int>* CostV) { }
    virtual void GetPredsofChangedEdges(std::vector<nav2dcell_t> const * changedcellsV, std::vector<int> *preds_of_changededgesIDV) { }
    virtual void GetSuccsofChangedEdges(std::vector<nav2dcell_t> const * changedcellsV, std::vector<int> *succs_of_changededgesIDV) { }
    virtual void InitializeEnvironment() { }
    virtual void GetSuccs(int SourceStateID, std::vector<int>* SuccIDV, std::vector<int>* CostV, std::vector<EnvNAVXYTHETALATAction_t*>* actionindV ) { }
    virtual void GetLazySuccs(int SourceStateID, std::vector<int>* SuccIDV, std::vector<int>* CostV, std::vector<bool>* isTrueCost, std::vector<EnvNAVXYTHETALATAction_t*>* actionindV ) { }
    virtual void GetSuccsWithUniqueIds(int SourceStateID, std::vector<int>* SuccIDV, std::vector<int>* CostV, std::vector<EnvNAVXYTHETALATAction_t*>* actionindV ) { }
    virtual void GetLazySuccsWithUniqueIds(int SourceStateID, std::vector<int>* SuccIDV, std::vector<int>* CostV, std::vector<bool>* isTrueCost, std::vector<EnvNAVXYTHETALATAction_t*>* actionindV ) { }
    virtual int GetTrueCost(int parentID, int childID) { }
    virtual bool isGoal(int id) { }
};

unsigned int*** allocateCostmap(const EnvNAVXYTHETALATConfig_t & cfg)
{
    unsigned int*** cost_map;
    cost_map = new unsigned int**[cfg.EnvWidth_c];
    for(unsigned int x = 0; x < cfg.EnvWidth_c; ++x) {
        cost_map[x] = new unsigned int*[cfg.EnvHeight_c];
        for(unsigned int y = 0; y < cfg.EnvHeight_c; ++y) {
            cost_map[x][y] = new unsigned int[cfg.NumThetaDirs];
            for(unsigned int th = 0; th < cfg.NumThetaDirs; ++th) {
                cost_map[x][y][th] = INFINITECOST;
            }
        }
    }
    return cost_map;
}

struct SearchNode {
    int x;
    int y;
    int theta;
    int g;

    bool operator<(const SearchNode & rhs) const {
        return g < rhs.g;
    }

    SearchNode(int x, int y, int theta, int g) : x(x), y(y), theta(theta), g(g) {}
};

void updateMaxCost(const EnvNAVXYTHETALATConfig_t & cfg, unsigned int*** costmap)
{
    int max = 0;
    for(unsigned int x = 0; x < cfg.EnvWidth_c; ++x) {
        for(unsigned int y = 0; y < cfg.EnvHeight_c; ++y) {
            for(unsigned int th = 0; th < cfg.NumThetaDirs; ++th) {
                if(costmap[x][y][th] == INFINITECOST)
                    continue;
                if(costmap[x][y][th] > max)
                    max = costmap[x][y][th];
            }
        }
    }
    if(max > g_maxCost || g_maxCost == INFINITECOST)
        g_maxCost = max;
}


void saveCostmap(const EnvNAVXYTHETALATConfig_t & cfg, int theta, unsigned int*** costmap, const std::string & filename)
{
    std::ofstream f(filename.c_str());
    if(!f.good())
        return;

    f << "P2 " << cfg.EnvWidth_c << " " << cfg.EnvHeight_c << " 255" << std::endl;

    for(unsigned int x = 0; x < cfg.EnvWidth_c; ++x) {
        for(unsigned int y = 0; y < cfg.EnvHeight_c; ++y) {
            unsigned int cost = costmap[x][y][theta];
            if(cost == INFINITECOST) {
                f << "255 0";
                continue;
            }
            int c = int(double(cost)/g_maxCost * 255.0);
            f << c << " ";
        }
        f << std::endl;
    }

    f.close();
}

void computeCosts(const EnvNAVXYTHETALATConfig_t & cfg, int theta, unsigned int*** costmap)
{
    std::priority_queue<SearchNode> queue;
    queue.push(SearchNode(cfg.EnvWidth_c/2, cfg.EnvHeight_c/2, theta, 0));

    unsigned int grid_size = cfg.EnvWidth_c * cfg.EnvHeight_c * cfg.NumThetaDirs;
    // TODO why is queue maxed out and doesn't change any more?

    unsigned int expanded = 0;
    while(!queue.empty()) {
        unsigned int debugInterval = 1;
        if(expanded % (debugInterval) == 0) {
            printf("Queue: %zu, Expanded: %d Grid: %d (%.2f x)\n", queue.size(), expanded, cfg.EnvWidth_c * cfg.EnvHeight_c * cfg.NumThetaDirs, (double)expanded/(cfg.EnvWidth_c * cfg.EnvHeight_c * cfg.NumThetaDirs)*1.0);
            unsigned int filled = 0;
            for(unsigned int x = 0; x < cfg.EnvWidth_c; ++x) {
                for(unsigned int y = 0; y < cfg.EnvHeight_c; ++y) {
                    for(unsigned int th = 0; th < cfg.NumThetaDirs; ++th) {
                        if(costmap[x][y][th] < INFINITECOST)
                            filled++;
                    }
                }
            }
            printf("Filled: %d/%d (%.2f %%)\n", filled, grid_size, double(filled)/grid_size*100.0);
            if(filled > 0) {
                if(g_maxCost == INFINITECOST)
                    updateMaxCost(cfg, costmap);
                static int count = 0;
                for(int th = 0; th < cfg.NumThetaDirs; th++) {
                    std::stringstream ss;
                    ss << std::setfill('0') << std::setw(6) << count;
                    std::string countStr = ss.str();
                    ss.str("");
                    ss << std::setfill('0') << std::setw(2) << th;
                    std::string endTh = ss.str();
                    ss.str("");
                    ss << std::setfill('0') << "costmap_" << std::setw(2) << theta << "_" << countStr << "_"  << endTh << ".pgm";
                    saveCostmap(cfg, th, costmap, ss.str());
                }
                count++;
                if(count > 10)
                    break;
            }
        }
        SearchNode current = queue.top();
        queue.pop();

        if(current.g >= costmap[current.x][current.y][current.theta])
            continue;
        costmap[current.x][current.y][current.theta] = current.g;

        expanded++;
        for(unsigned int i = 0; i < cfg.actionwidth; ++i) {
            const EnvNAVXYTHETALATAction_t & act = cfg.ActionsV[current.theta][i];
            SearchNode succ(current.x + act.dX, current.y + act.dY, act.endtheta, current.g + act.cost);
            if(succ.x < 0 || succ.x >= cfg.EnvWidth_c || succ.y < 0 || succ.y >= cfg.EnvHeight_c)
                continue;
            if(succ.g < costmap[succ.x][succ.y][succ.theta])
                queue.push(succ);
        }
    }
    printf("Total expanded: %d\n", expanded);
}

int main(int argc, char** argv)
{
    std::string mprims_file;
    int size_x, size_y;
    double nominalvel_mpersecs;
    double timetoturn45degsinplace_secs;

    int c;
    while(true) {
        static struct option long_options[] = {
            {"sx", required_argument, 0, 'x'},
            {"sy", required_argument, 0, 'y'},
            {"mprims", required_argument, 0, 'f'},
            {"tv", required_argument, 0, 't'},
            {"rv", required_argument, 0, 'r'},
            {0, 0, 0, 0}
        };
        int option_index = 0;
        c = getopt_long(argc, argv, "x:y:f:t:r:", long_options, &option_index);
        if(c == -1)
            break;
        switch (c) {
            case 'x':
                size_x = atoi(optarg);
                break;
            case 'y':
                size_y = atoi(optarg);
                break;
            case 'f':
                mprims_file = optarg;
                break;
            case 't':
                nominalvel_mpersecs = atof(optarg);
                break;
            case 'r':
                timetoturn45degsinplace_secs = atof(optarg);
                break;
        }
    }

    printf("Grid (%d x %d), tv: %.2f rv: %.2f\n", size_x, size_y, nominalvel_mpersecs, timetoturn45degsinplace_secs);
    printf("Reading prims from %s\n", mprims_file.c_str());

    EnvironmentMotionPrims env(size_x, size_y, nominalvel_mpersecs, timetoturn45degsinplace_secs);
    if(!env.readPrimitives(mprims_file)) {
        fprintf(stderr, "Failed to read prims.\n");
        return 1;
    }

    const EnvNAVXYTHETALATConfig_t& cfg = env.getConfig();
    std::vector<unsigned int ***> costmaps; // one costmap per start theta
    for(unsigned int i = 0; i < cfg.NumThetaDirs; ++i) {
        costmaps.push_back(allocateCostmap(cfg));
    }
    // Each costmap contains the costs to get from (0, 0, start theta) -> each cells (dx, dy, end theta)

    // TODO parallelize
    for(unsigned int th = 0; th < cfg.NumThetaDirs; ++th) {
        printf("Computing costs for th: %d\n", th);
        g_maxCost = INFINITECOST;
        computeCosts(cfg, th, costmaps[th]);
        break;
    }

    return 0;
}

