// Thin ABI to the reference AWX geometry backend (SciPy linprog(method="highs")).
// Every variable, row and objective coefficient is assembled in Rust.
#include <cstdint>
#include <memory>
#include <vector>
#include "highs/interfaces/highs_c_api.h"
extern "C" int awx_highs_solve(int n,int m,const double* lower,const double* upper,
 const double* cost,const int* starts,const int* columns,const double* values,
 const double* row_lower,const double* row_upper,int64_t milliseconds,
 double* solution,double* row_dual,double* objective,int mode){
 try{
  auto destroy=[](void* h){Highs_destroy(h);};
  std::unique_ptr<void,decltype(destroy)> highs(Highs_create(),destroy);
  if(!highs)return -1;
  Highs_setBoolOptionValue(highs.get(),"output_flag",0);
  Highs_setDoubleOptionValue(highs.get(),"time_limit",milliseconds/1000.0);
  if(mode==1){Highs_setStringOptionValue(highs.get(),"solver","ipm");Highs_setIntOptionValue(highs.get(),"ipm_iteration_limit",500);}
  if(mode==2)Highs_setStringOptionValue(highs.get(),"solver","simplex");
  Highs_setDoubleOptionValue(highs.get(),"primal_feasibility_tolerance",1e-7);
  Highs_setDoubleOptionValue(highs.get(),"dual_feasibility_tolerance",1e-7);
  std::vector<HighsInt> row_starts(starts,starts+m+1),indices(columns,columns+starts[m]);
  if(Highs_passLp(highs.get(),n,m,starts[m],kHighsMatrixFormatRowwise,kHighsObjSenseMinimize,0.,cost,lower,upper,row_lower,row_upper,row_starts.data(),indices.data(),values)<0)return -3;
  if(Highs_run(highs.get())<0)return -4;
  auto status=Highs_getModelStatus(highs.get());HighsInt primal=0;
  Highs_getIntInfoValue(highs.get(),"primal_solution_status",&primal);
  if(status==kHighsModelStatusOptimal||primal==kHighsSolutionStatusFeasible){Highs_getSolution(highs.get(),solution,nullptr,nullptr,row_dual);*objective=Highs_getObjectiveValue(highs.get());return status==kHighsModelStatusOptimal?0:1;}
  return 100+static_cast<int>(status);
 }catch(...){return -2;}
}
