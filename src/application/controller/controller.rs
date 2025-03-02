pub struct ControllerApplication<S, F>
where
    S: StateVector,
    F: Force,
{
    controller: Box<dyn Controller<S, F>>,
}

impl<S, F> ControllerApplication<S, F>
where
    S: StateVector,
    F: Force,
{
    pub fn compute_control_input(&self, state: &S) -> F {
        self.controller.compute_control_input(state)
    }
}
