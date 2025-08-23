# Firmware Timeout Update Status - PH2K Project

## Overview
This document tracks the timeout configuration status across all firmware files. The target is to ensure all CAN communication timeouts are set to **10000ms (10 seconds)** for improved reliability.

## ✅ Firmware Files with Correct 10000ms Timeout

### Already Updated and Working
- **✅ hose_jig.ino** - Timeout: 10000ms (Line 830)
- **✅ hose_puller.ino** - Timeout: 10000ms (Line 919)
- **✅ lubrication_feeder.ino** - Timeout: 10000ms (Line 1280)

### Partially Updated (Mixed Timeouts)
- **⚠️ elevator_in.ino** - Has both 10000ms (Line 697) and 6000ms (Line 728)
- **⚠️ pick_and_place.ino** - Has both 10000ms (Line 917) and 6000ms (Line 943)

## ❌ Firmware Files Requiring Updates

### Need Complete Timeout Updates to 10000ms

#### 1. elevator_out.ino
- **Status**: ❌ All timeouts are 6000ms
- **Lines to Update**: 582, 653, 768, 799
- **Action Required**: Change all `timeout = 6000` to `timeout = 10000`

#### 2. insertion_jig.ino
- **Status**: ❌ All timeouts are 6000ms
- **Lines to Update**: 720, 748
- **Action Required**: Change all `timeout = 6000` to `timeout = 10000`

### Need Partial Updates (Complete Remaining Timeouts)

#### 3. elevator_in.ino
- **Status**: ⚠️ Partially updated
- **Line to Update**: 728
- **Action Required**: Change `timeout = 6000` to `timeout = 10000`

#### 4. pick_and_place.ino
- **Status**: ⚠️ Partially updated
- **Line to Update**: 943
- **Action Required**: Change `timeout = 6000` to `timeout = 10000`

### Special Case - Different Timeout Mechanism

#### 5. insertion_servos.ino
- **Status**: ⚠️ Uses dynamic timeout calculation
- **Current Logic**: `timeout = instruction.data[1] > 0 ? instruction.data[1] * 100 : 1000`
- **Line**: 414
- **Action Required**: Review if this dynamic timeout is appropriate or needs adjustment

## 📋 Upload Priority List

### High Priority (Complete Updates Needed)
1. **elevator_out.ino** - 4 timeout values to update
2. **insertion_jig.ino** - 2 timeout values to update

### Medium Priority (Partial Updates Needed)
3. **elevator_in.ino** - 1 remaining timeout value to update
4. **pick_and_place.ino** - 1 remaining timeout value to update

### Review Required
5. **insertion_servos.ino** - Evaluate dynamic timeout mechanism

## 🔧 Quick Reference for Updates

### Search and Replace Pattern
```cpp
// Find:
const unsigned long timeout = 6000;

// Replace with:
const unsigned long timeout = 10000;
```

### Files Not Requiring Updates
- **hose_jig.ino** ✅
- **hose_puller.ino** ✅
- **lubrication_feeder.ino** ✅
- **safety.ino** (No timeout configurations found)
- **puller_extension.ino** (No timeout configurations found)
- **power_system files** (No timeout configurations found)

## 📝 Notes

- **Template files** (`template.ino`, `template copy.ino`) use dynamic timeout calculation similar to `insertion_servos.ino`
- **Backup files** in `/bk/` directory don't require updates
- All timeout changes should include the `flushCanBuffer()` implementation for complete CAN communication reliability

## 🚀 Deployment Checklist

Before uploading to microcontrollers:
1. ✅ Verify timeout values are set to 10000ms
2. ✅ Ensure `flushCanBuffer()` function is implemented
3. ✅ Confirm `flushCanBuffer()` is called at the beginning of `process_instruction()`
4. ✅ Test CAN communication after upload
5. ✅ Update this document after successful deployment

---
*Last Updated: Based on firmware analysis - All timeout configurations identified and categorized*