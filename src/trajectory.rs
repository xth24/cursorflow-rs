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
        self.x.len()
    }

    pub fn is_empty(&self) -> bool {
        self.x.is_empty()
    }

    pub fn deduplicate(&self) -> Trajectory {
        let mut out = Trajectory::empty();
        for i in 0..self.len() {
            if i == 0 || self.x[i] != self.x[i - 1] || self.y[i] != self.y[i - 1] {
                out.x.push(self.x[i]);
                out.y.push(self.y[i]);
                out.t.push(self.t[i]);
                out.v.push(self.v[i]);
            }
        }
        out
    }
}
