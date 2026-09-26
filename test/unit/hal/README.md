
### HAL Testing

Due to the way that the HAL object is constructed (global instance of derived with extern const base-class reference) we need to have separate executables for each HAL test.

We have the following HAL tests:

- HAL Initialisation succeeds
- HAL GZ Interface can send each type of message
- Simulator can communicate with HAL
- Simulator and HAL can run n iterations via tick

- HAL/Simulator can timeout effectively

- HAL/Simulator can re-connect effectively


