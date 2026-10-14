// Native ABI boundary only. Models and all stopping decisions are owned by Rust.
#include <atomic>
#include <climits>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <mutex>
#include <string>
#include "ortools/sat/cp_model_solver.h"
#include "ortools/sat/model.h"
#include "ortools/util/logging.h"
#include "ortools/util/time_limit.h"
extern "C" {
typedef bool (*awx_log_callback)(const char*, size_t, void*);
int awx_solve_cp_sat(const uint8_t* request, size_t request_len,
 const uint8_t* parameters, size_t parameters_len, awx_log_callback callback,
 void* context, uint8_t** response, size_t* response_len) noexcept {
 if (!response || !response_len) return -1;
 *response=nullptr; *response_len=0;
 if (!request || !parameters || request_len>INT_MAX || parameters_len>INT_MAX) return -2;
 try {
  operations_research::sat::CpModelProto proto;
  operations_research::sat::SatParameters params;
  if (!proto.ParseFromArray(request,static_cast<int>(request_len)) || !params.ParseFromArray(parameters,static_cast<int>(parameters_len))) return -3;
  std::atomic<bool> stopped(false);
  std::mutex callback_mutex;
  operations_research::sat::Model model;
  model.Add(operations_research::sat::NewSatParameters(params));
  model.GetOrCreate<operations_research::TimeLimit>()->RegisterExternalBooleanAsLimit(&stopped);
  if (callback) {
   model.GetOrCreate<operations_research::SolverLogger>()->AddInfoLoggingCallback([&](const std::string& message) {
    std::lock_guard<std::mutex> lock(callback_mutex);
    if (callback(message.data(),message.size(),context)) stopped.store(true,std::memory_order_relaxed);
   });
  }
  const auto result=operations_research::sat::SolveCpModel(proto,&model);
  std::string encoded;
  if (!result.SerializeToString(&encoded)) return -4;
  if (!encoded.empty()) {
   auto* buffer=static_cast<uint8_t*>(std::malloc(encoded.size()));
   if (!buffer) return -5;
   std::memcpy(buffer,encoded.data(),encoded.size()); *response=buffer;
  }
  *response_len=encoded.size(); return 0;
 } catch (...) { return -6; }
}
void awx_cp_sat_free(void* response) noexcept {std::free(response);}
}
