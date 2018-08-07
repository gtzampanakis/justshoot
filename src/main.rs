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
    JGVector3,
    JUnitQuaternion,
};

struct GraphicsConf {
    width: u32,
    height: u32,
    pixels_per_meter: f32,
    origin: JGVector3,
}

impl GraphicsConf {
    fn pos_to_graph(&self, v: &JVector3) -> JGVector3 {
        JGVector3::new(
            (v.x as f32) * self.pixels_per_meter,
            (v.y as f32) * self.pixels_per_meter,
            (v.z as f32) * self.pixels_per_meter,
        )
    }
}


struct GameState {
    graphics_conf: GraphicsConf,
    simulator: Simulator,
    shot_done: bool,
    simulation_state_seq: SimulationStateSeq,
    t_shot_start: Option<f64>,
    simulation_state: Option<SimulationState>,
}

impl GameState {
    fn new() -> Self {
        let graphics_conf = GraphicsConf {
            width: 640,
            height: 480,
            pixels_per_meter: 800.,
            origin: JGVector3::new(640./2., 480./2., 0.),
        };

        let world_conf = WorldConf {
            ball_radius: consts::POOL_BALL_RADIUS,
            ball_weight: consts::POOL_BALL_WEIGHT,
        };

        let balls = vec![
            Ball {
                pos: JVector3::new(-0.5, 0., 0.),
                urot: JUnitQuaternion::identity(),
                u: JVector3::new(0.5, 0., 0.),
                rot: JUnitQuaternion::identity(),
            },
            Ball {
                pos: JVector3::new(0.0, 0.0475, 0.),
                urot: JUnitQuaternion::identity(),
                u: JVector3::new(0., 0., 0.),
                rot: JUnitQuaternion::identity(),
            },
            Ball {
                pos: JVector3::new(0.2, 0.0875, 0.),
                urot: JUnitQuaternion::identity(),
                u: JVector3::new(0., 0., 0.),
                rot: JUnitQuaternion::identity(),
            },
            Ball {
                pos: JVector3::new(0.0, -0.0575, 0.),
                urot: JUnitQuaternion::identity(),
                u: JVector3::new(0., 0., 0.),
                rot: JUnitQuaternion::identity(),
            },
        ];
        
        GameState {
            graphics_conf: graphics_conf,
            simulator: Simulator::new(
                balls,
                world_conf,
                1e-4,
            ),
            shot_done: false,
            t_shot_start: None,
            simulation_state_seq: SimulationStateSeq { states: Vec::with_capacity(2) },
            simulation_state: None,
        }
    }
}

impl event::EventHandler for GameState {
  fn update(&mut self, ctx: &mut Context) -> GameResult<()> {
    const DESIRED_FPS: u32 = 60;

    while timer::check_update_time(ctx, DESIRED_FPS) {

        let t = timer::duration_to_f64(timer::get_time_since_start(ctx));

        // Start the simulation right away.
        if let None = self.t_shot_start {
            self.t_shot_start = Some(t);
        }

        if let Some(t_shot_start) = self.t_shot_start {

            let t_elapsed_in_shot = t - t_shot_start;

            while self.simulator.t < t_elapsed_in_shot {
                let simulation_state = self.simulator.progress();

                // for ball in self.simulator.balls.iter() {
                //     println!("{:?}", ball.pos);
                // }
                // println!("");

                if self.simulation_state_seq.states.len() == 2 {
                    self.simulation_state_seq.states.remove(0);
                }
                self.simulation_state_seq.states.push(simulation_state);
            }

            if !self.simulation_state_seq.states.is_empty() {
                self.simulation_state = Some(
                    self.simulation_state_seq.calc_interpolated_at(t_elapsed_in_shot));
            }

            println!("fps: {:?}", timer::get_fps(ctx));

        }

    }

    Ok(())
  }

  fn draw(&mut self, ctx: &mut Context) -> GameResult<()> {
    graphics::clear(ctx);

    if let Some(ref simulation_state) = self.simulation_state {
        for ball in self.simulator.balls.iter() {
            let ball_graphic = graphics::circle(
                ctx,
                graphics::DrawMode::Fill,
                graphics::Point2::new(
                    self.graphics_conf.origin.x + (ball.pos.x as f32) * self.graphics_conf.pixels_per_meter,
                    self.graphics_conf.origin.y + (ball.pos.y as f32) * self.graphics_conf.pixels_per_meter,
                ),
                (self.simulator.world_conf.ball_radius as f32) * self.graphics_conf.pixels_per_meter
                                                               * (1. + ball.pos.z as f32),
                0.01,
            );
        }
    }

    graphics::present(ctx);
    timer::yield_now();

    Ok(())
  }
}

fn show() {
    let mut game_state = GameState::new();

    let mut cb = ContextBuilder::new("justshoot", "gtz")
        .window_setup(conf::WindowSetup::default().title("justshoot"))
        .window_mode(conf::WindowMode::default().dimensions(
            game_state.graphics_conf.width, game_state.graphics_conf.height)
        );
    let ctx = &mut cb.build().unwrap();

    event::run(ctx, &mut game_state);
}

fn main() {
    show();
}
