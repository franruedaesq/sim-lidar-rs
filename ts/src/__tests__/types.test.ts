import { describe, it, expect } from "vitest";
import {
  VLP16_CONFIG,
  OUSTER_OS1_32_CONFIG,
  OUSTER_OS1_64_CONFIG,
  totalRays,
  type SensorConfig,
} from "../types.js";

describe("SensorConfig presets", () => {
  it("VLP16_CONFIG has correct total rays", () => {
    expect(totalRays(VLP16_CONFIG)).toBe(1800 * 16);
  });

  it("OUSTER_OS1_32_CONFIG has correct total rays", () => {
    expect(totalRays(OUSTER_OS1_32_CONFIG)).toBe(1024 * 32);
  });

  it("OUSTER_OS1_64_CONFIG has correct total rays", () => {
    expect(totalRays(OUSTER_OS1_64_CONFIG)).toBe(2048 * 64);
  });

  it("VLP16_CONFIG vertical FOV is symmetric ±15°", () => {
    expect(VLP16_CONFIG.verticalFovUpper).toBe(15);
    expect(VLP16_CONFIG.verticalFovLower).toBe(-15);
  });

  it("all presets have valid range limits", () => {
    const presets: Readonly<SensorConfig>[] = [VLP16_CONFIG, OUSTER_OS1_32_CONFIG, OUSTER_OS1_64_CONFIG];
    for (const cfg of presets) {
      expect(cfg.minRange).toBeGreaterThan(0);
      expect(cfg.maxRange).toBeGreaterThan(cfg.minRange);
    }
  });

  it("totalRays returns horizontalResolution * verticalChannels", () => {
    const cfg: SensorConfig = {
      horizontalResolution: 360,
      verticalChannels: 4,
      verticalFovUpper: 10,
      verticalFovLower: -10,
      minRange: 0.5,
      maxRange: 50,
      noiseStddev: 0,
    };
    expect(totalRays(cfg)).toBe(360 * 4);
  });
});
