#[derive(Debug, Clone, Copy)]
pub struct ScreenConfig {
    pub width: u32,
    pub height: u32,
    pub sample_rate: u32,
}

impl Default for ScreenConfig {
    fn default() -> Self {
        Self {
            width: 1920,
            height: 1080,
            sample_rate: 120,
        }
    }
}

impl ScreenConfig {
    pub fn diagonal(&self) -> f64 {
        ((self.width as f64).powi(2) + (self.height as f64).powi(2)).sqrt()
    }
}
