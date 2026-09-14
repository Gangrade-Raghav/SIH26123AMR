# M9 Multi-Scenario Empirical Comparison Summary

| Scenario | Fleet | Workload | Blockage | Comm Loss | Initial CBBA | Dynamic CBBA | Replans | Mean Latency | Min Dist | Contacts | OBB Overlaps | Brakes | Tasks (A / IP / C) |
| :--- | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: |
| M9-V1 (Expanded 5 AMR) | 5 | 15 | None | 0.0% | 1322.7 ms | N/A | 255 | 0.14 ms | 4.44 m | 0 | 0 | 39 | 15A / 0IP / 0C |
| M9-V2 (Congested 10 AMR) | 10 | 30 | None | 0.0% | 4.5 ms | N/A | 492 | 0.14 ms | 2.05 m | 0 | 0 | 3 | 29A / 0IP / 0C |
| M9-V3-D (Dynamic Arrival) | 10 | 30 | None | 0.0% | 8.3 ms | 2875.3 ms | 423 | 0.27 ms | 5.45 m | 0 | 0 | 5 | 27A / 2IP / 1C |
| M9-V3-A (Dynamic Blockage) | 10 | 30 | Yes [45-90s] | 0.0% | 8.5 ms | 2695.9 ms | 454 | 0.23 ms | 5.45 m | 0 | 0 | 5 | 28A / 2IP / 0C |
| M9-V3-E (Combined Stress) | 10 | 30 | Yes [45-90s] | 8.58% | 14.0 ms | 167.6 ms | 454 | 0.31 ms | 5.45 m | 0 | 0 | 4 | 28A / 2IP / 0C |
