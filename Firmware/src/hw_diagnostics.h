#ifndef HW_DIAGNOSTICS_H
#define HW_DIAGNOSTICS_H

#include <stdint.h>
#include <stddef.h>

// Run full hardware diagnostics (call from loopTask only)
void runHardwareDiagnostics();

// True while diagnostics are running
bool isHWDiagRunning();

// Serve diagnostic report as chunked JSON (called from web_server.cpp)
// Returns true if more chunks remain, false when done.
// Call with section starting at 0, incrementing after each call.
bool serveHWDiagChunk(int section, char* buf, size_t bufSize, size_t* written);

// Total number of report sections (for chunked response)
int getHWDiagSectionCount();

// True if a report is available for download
bool isHWDiagReportReady();

#endif // HW_DIAGNOSTICS_H
