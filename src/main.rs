extern crate ggez;
use ggez::*;

mod jlib;
mod geometry;

use jlib::{
    Ball,
    Simulator,
    SimulationState,
    SimulationStateSeq,
    WorldConf,
    consts,
};

use geometry::{
    JVector3,
    JUnitQuaternion,
};

struct GameState {
    simulator: Simulator,
    shot_done: bool,
    simulation_state_seq: SimulationStateSeq,
    t_shot_start: Option<f64>,
    simulation_state: Option<SimulationState>,
}

impl GameState {
    fn new() -> Self {
        let world_conf = WorldConf {
            ball_radius: consts::POOL_BALL_RADIUS,
            ball_weight: consts::POOL_BALL_WEIGHT,
        };

        let balls = vec![
            Ball {
                pos: JVector3::zeros(),
                urot: JUnitQuaternion::identity(),
                u: JVector3::new(20., 0., 0.),
                rot: JUnitQuaternion::identity(),
            },
            Ball {
                pos: JVector3::new(40., 0.0025, 0.),
                urot: JUnitQuaternion::identity(),
                u: JVector3::new(0., 0., 0.),
                rot: JUnitQuaternion::identity(),
            },
        ];
        
        GameState {
            simulator: Simulator::new(
                balls,
                world_conf,
                1e-3,
            ),
            shot_done: false,
            t_shot_start: None,
            simulation_state_seq: SimulationStateSeq { states: vec![] },
            simulation_state: None,
        }
    }
}

impl event::EventHandler for GameState {
  fn update(&mut self, ctx: &mut Context) -> GameResult<()> {
    const DESIRED_FPS: u32 = 60;

    let t = timer::duration_to_f64(timer::get_time_since_start(ctx));

    // Start the simulation right away.
    if let None = self.t_shot_start {
        self.t_shot_start = Some(t);
    }

    if let Some(t_shot_start) = self.t_shot_start {

        let t_elapsed_in_shot = t - t_shot_start;

        while timer::check_update_time(ctx, DESIRED_FPS) {
            while self.simulator.t < t_elapsed_in_shot {
                let simulation_state = self.simulator.progress();
                self.simulation_state_seq.states.push(simulation_state);
            }

            if !self.simulation_state_seq.states.is_empty() {
                self.simulation_state = Some(
                    self.simulation_state_seq.calc_interpolated_at(t_elapsed_in_shot));
            }

            println!("{:?}", t);
        }

    }

    Ok(())
  }

  fn draw(&mut self, ctx: &mut Context) -> GameResult<()> {
    graphics::clear(ctx);

    if let Some(ref simulation_state) = self.simulation_state {
        for ball in self.simulator.balls.iter() {
            println!("ball pos: {:?}", ball.pos);
            let ball_graphic = graphics::circle(
                ctx,
                graphics::DrawMode::Fill,
                graphics::Point2::new((200.0 + ball.pos.x as f32), (300.0 + ball.pos.y as f32)),
                5.0,
                0.1,
            );
        }
    }

    graphics::present(ctx);

    Ok(())
  }
}

fn show() {
    let mut game_state = GameState::new();

    let mut cb = ContextBuilder::new("justshoot", "gtz")
        .window_setup(conf::WindowSetup::default().title("justshoot"))
        .window_mode(conf::WindowMode::default().dimensions(640, 480));
    let ctx = &mut cb.build().unwrap();

    event::run(ctx, &mut game_state);
}

fn main() {
    show();
}
