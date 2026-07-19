#include "static_snapshot.h"

#include <cstdint>
#include <cstdio>
#include <cstring>
#include <cstdlib>

#if defined(__APPLE__)
#include <mach-o/dyld.h>
#include <mach-o/loader.h>
#include <mach-o/ldsyms.h>
#include <mach/vm_prot.h>
#elif defined(_WIN32)
#include <windows.h>
#endif

namespace SimITL {

  namespace {

    struct Region {
      uintptr_t addr = 0;
      size_t size = 0;
      uint8_t* copy = nullptr; // heap: unaffected by the restore itself
    };

    struct Exclusion {
      uintptr_t addr = 0;
      size_t size = 0;
    };

    constexpr int MAX_REGIONS = 64;
    constexpr int MAX_EXCLUSIONS = 16;

    /* All bookkeeping lives in this ONE static struct so it can exclude
     * itself from the restore — otherwise the restore would rewind its own
     * region table mid-iteration. The copies are heap allocations and are
     * never touched by a restore. */
    struct State {
      Region regions[MAX_REGIONS];
      int regionCount = 0;
      Exclusion exclusions[MAX_EXCLUSIONS];
      int exclusionCount = 0;
      bool taken = false;
    };
    State g;

    /* Enumerate the writable static sections of the main image.
     * Returns false when the platform walk is not implemented. */
    template <typename Fn>
    bool forEachWritableSection(Fn&& fn)
    {
#if defined(__APPLE__)
      const struct mach_header_64* mh = &_mh_execute_header;
      intptr_t slide = 0;
      for (uint32_t i = 0; i < _dyld_image_count(); i++) {
        if (_dyld_get_image_header(i) == (const struct mach_header*)mh) {
          slide = _dyld_get_image_vmaddr_slide(i);
          break;
        }
      }
      const struct load_command* lc = (const struct load_command*)(mh + 1);
      for (uint32_t i = 0; i < mh->ncmds; i++) {
        if (lc->cmd == LC_SEGMENT_64) {
          const struct segment_command_64* seg =
              (const struct segment_command_64*)lc;
          /* __DATA only: __DATA_CONST is re-protected read-only after dyld
           * fixups (writing would fault) and never mutates afterwards. */
          if ((seg->initprot & VM_PROT_WRITE)
              && strcmp(seg->segname, "__DATA") == 0) {
            const struct section_64* sec = (const struct section_64*)(seg + 1);
            for (uint32_t s = 0; s < seg->nsects; s++, sec++) {
              fn((uintptr_t)sec->addr + slide, (size_t)sec->size);
            }
          }
        }
        lc = (const struct load_command*)((const uint8_t*)lc + lc->cmdsize);
      }
      return true;
#elif defined(__linux__)
      extern char __data_start[], _edata[], __bss_start[], _end[];
      fn((uintptr_t)__data_start, (size_t)(_edata - __data_start));
      fn((uintptr_t)__bss_start, (size_t)(_end - __bss_start));
      return true;
#elif defined(_WIN32)
      const uint8_t* base = (const uint8_t*)GetModuleHandleA(NULL);
      const IMAGE_DOS_HEADER* dos = (const IMAGE_DOS_HEADER*)base;
      const IMAGE_NT_HEADERS* nt =
          (const IMAGE_NT_HEADERS*)(base + dos->e_lfanew);
      const IMAGE_SECTION_HEADER* sec = IMAGE_FIRST_SECTION(nt);
      for (unsigned i = 0; i < nt->FileHeader.NumberOfSections; i++, sec++) {
        if ((sec->Characteristics & IMAGE_SCN_MEM_WRITE)
            && !(sec->Characteristics & IMAGE_SCN_MEM_DISCARDABLE)) {
          size_t size = sec->Misc.VirtualSize;
          fn((uintptr_t)(base + sec->VirtualAddress), size);
        }
      }
      return true;
#else
      return false;
#endif
    }

  } // namespace

  void snapshotExclude(void* addr, size_t size)
  {
    if (g.taken) {
      printf("[pw][snapshot] exclusion after snapshot — ignored\n");
      return;
    }
    if (g.exclusionCount >= MAX_EXCLUSIONS) {
      printf("[pw][snapshot] exclusion table full — ignored\n");
      return;
    }
    g.exclusions[g.exclusionCount++] = { (uintptr_t)addr, size };
  }

  bool snapshotTake()
  {
    if (g.taken) return true; // first boot is the canonical state

    // the bookkeeping must never restore over itself
    snapshotExclude(&g, sizeof(g));

    bool ok = forEachWritableSection([](uintptr_t addr, size_t size) {
      if (size == 0) return;
      if (g.regionCount >= MAX_REGIONS) {
        printf("[pw][snapshot] region table full — section dropped\n");
        return;
      }
      Region& r = g.regions[g.regionCount++];
      r.addr = addr;
      r.size = size;
      r.copy = (uint8_t*)malloc(size);
      memcpy(r.copy, (const void*)addr, size);
    });

    if (!ok) {
      printf("[pw][snapshot] unsupported platform — reset stays init()-based\n");
      return false;
    }
    g.taken = true;
    size_t total = 0;
    for (int i = 0; i < g.regionCount; i++) total += g.regions[i].size;
    printf("[pw][snapshot] captured %d section(s), %zu KiB, %d exclusion(s)\n",
           g.regionCount, total / 1024, g.exclusionCount);
    return true;
  }

  bool snapshotRestore()
  {
    if (!g.taken) return false;

    for (int i = 0; i < g.regionCount; i++) {
      const Region& r = g.regions[i];
      /* restore in pieces, skipping the exclusion holes */
      uintptr_t pos = r.addr;
      const uintptr_t end = r.addr + r.size;
      while (pos < end) {
        /* the next exclusion that overlaps [pos, end) */
        uintptr_t holeStart = end;
        uintptr_t holeEnd = end;
        for (int e = 0; e < g.exclusionCount; e++) {
          const uintptr_t es = g.exclusions[e].addr;
          const uintptr_t ee = es + g.exclusions[e].size;
          if (ee > pos && es < holeStart) {
            holeStart = es < pos ? pos : es;
            holeEnd = ee > end ? end : ee;
          }
        }
        if (holeStart > pos) {
          memcpy((void*)pos, r.copy + (pos - r.addr), holeStart - pos);
        }
        pos = holeEnd > pos ? holeEnd : pos + 1;
      }
    }
    return true;
  }

  bool snapshotTaken()
  {
    return g.taken;
  }

}
