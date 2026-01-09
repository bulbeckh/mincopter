### Dynamics
From https://arxiv.org/pdf/1611.09240

$$\begin{align}
\dot{\textbf{p}} &= \textbf{v}\\
\dot{\textbf{v}} &= \frac{1}{m}(\textbf{R}_{IB}\sum_{i=0}^{N_{r}}\textbf{F}_{T,i})+
\begin{bmatrix}
0 \\
0 \\
-g \\
\end{bmatrix}\\
\dot{\textbf{R}}_{IB} &= \textbf{R}_{IB}\lfloor\boldsymbol{\omega}\times\rfloor\\
\textbf{J}\dot{\boldsymbol{\omega}}&=-\boldsymbol{\omega}\times\textbf{J}\boldsymbol{\omega} + \textit{A}
\begin{bmatrix}
n_{1}^{2} \\
\vdots \\
n_{N_{r}}^{2} \\
\end{bmatrix}
\end{align}
$$

More detailed state space model, equivalent to the above from this paper: https://arxiv.org/pdf/2504.13286

We use a NED frame with **intrinsic** Y-P-R rotations ${}^{I}R_{B}=R_{\psi}R_{\theta}R_{\phi}$ with body frame and inertial frames aligned when there is **no rotation**.

$$
\begin{align}
m\ddot{X} &= -F(\cos{\phi}\sin{\theta}\cos{\psi}+\sin{\phi}\sin{\psi}) \\
m\ddot{Y} &= -F(\cos{\phi}\sin{\theta}\sin{\psi}-\sin{\phi}\cos{\psi}) \\
m\ddot{Z} &= -F(\cos{\phi}\cos{\theta})+mg \\
I_{x}\ddot{\phi} &= T_{x}l+\dot{\theta}\dot{\psi}(I_{y}-I_{z}) \\
I_{y}\ddot{\theta} &= T_{y}l+\dot{\psi}\dot{\phi}(I_{z}-I_{x}) \\
I_{z}\ddot{\psi} &= T_{z}l+\dot{\phi}\dot{\theta}(I_{x}-I_{y}) \\
F & = F_1 + F_2 + F_3 + F_4
\end{align}
$$

#### `TODO` Add noise to the state dynamics