#[derive(Debug, Default, Clone)]
pub struct Trajectory {
    pub x: Vec<i32>,
    pub y: Vec<i32>,
    pub t: Vec<f64>,
    pub v: Vec<f64>,
}

impl Trajectory {
    pub fn empty() -> Self {
        Self::default()
    }

    pub fn len(&self) -> usize {
        self.x
            .len()
            .min(self.y.len())
            .min(self.t.len())
            .min(self.v.len())
    }

    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }

    pub fn deduplicate(&self) -> Trajectory {
        let mut out = Trajectory::empty();
        let n = self.position_len();
        let mut last_point = None;

        for i in 0..n {
            let point = (self.x[i], self.y[i]);
            if last_point != Some(point) {
                out.x.push(self.x[i]);
                out.y.push(self.y[i]);
                out.t.push(self.t[i]);
                last_point = Some(point);
            }
        }

        out.refresh_velocities();
        out
    }

    pub fn recompute_velocities(&self) -> Trajectory {
        let n = self.position_len();
        let mut out = Trajectory {
            x: self.x[..n].to_vec(),
            y: self.y[..n].to_vec(),
            t: self.t[..n].to_vec(),
            v: Vec::new(),
        };
        out.refresh_velocities();
        out
    }

    pub(crate) fn refresh_velocities(&mut self) {
        let n = self.position_len();
        self.x.truncate(n);
        self.y.truncate(n);
        self.t.truncate(n);
        self.v = Self::velocities_for(&self.x, &self.y, &self.t);
    }

    pub(crate) fn velocities_for(x: &[i32], y: &[i32], t: &[f64]) -> Vec<f64> {
        let n = x.len().min(y.len()).min(t.len());
        let mut velocities = vec![0.0; n];

        for i in 1..n {
            let dt = (t[i] - t[i - 1]) / 1000.0;
            if dt.is_finite() && dt > 0.0 {
                let dx = (x[i] - x[i - 1]) as f64;
                let dy = (y[i] - y[i - 1]) as f64;
                velocities[i] = dx.hypot(dy) / dt;
            }
        }

        velocities
    }

    fn position_len(&self) -> usize {
        self.x.len().min(self.y.len()).min(self.t.len())
    }
}
