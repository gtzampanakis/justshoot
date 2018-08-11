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
    JUnitVector3,
    JGVector3,
    JQuaternion,
    JUnitQuaternion,
};

struct GraphicsConf {
    width: u32,
    height: u32,
    pixels_per_meter: f32,
    origin: JGVector3,
    eye_height: f64,
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
            eye_height: 2.5,
        };

        let world_conf = WorldConf {
            gravity: consts::GRAVITY,
            ball_radius: consts::POOL_BALL_RADIUS,
            ball_weight: consts::POOL_BALL_WEIGHT,
            ball_ball_rest: consts::BALL_BALL_REST,
            ball_cloth_rest: consts::BALL_CLOTH_REST,
            ball_spot_poss: vec![
                JUnitVector3::new_normalize(JVector3::new(0., 0., 1.)),
                JUnitVector3::new_normalize(JVector3::new(0., 0., -1.)),
            ],
            ball_spot_radius_factor: consts::BALL_SPOT_RADIUS_FACTOR,
        };

        let balls = vec![
            Ball {
                pos: JVector3::new(-0.2, 0.0875, 5.),
                urot_axis: JUnitVector3::new_normalize(JVector3::new(1., 0.5, 2.2)),
                urot_angle: 8. * 3.14,
                u: JVector3::new(0.125, 0., 0.) * 0.4,
                rot: JUnitQuaternion::identity(),
            },
            Ball {
                pos: JVector3::new(-0.1, 0.0875, 20.),
                urot_axis: JUnitVector3::new_normalize(JVector3::new(13., 0.4, 0.1)),
                urot_angle: 2. * 3.14,
                u: JVector3::new(0.0, 0.000, 0.) * 0.4,
                rot: JUnitQuaternion::identity(),
            },
            Ball {
                pos: JVector3::new(0.0, 0.0875, 25.),
                urot_axis: JUnitVector3::new_normalize(JVector3::new(1., 0.2, 0.)),
                urot_angle: 12. * 3.14,
                u: JVector3::new(-0.01625, 0.002, -8.) * 0.4,
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

                if self.simulation_state_seq.states.len() == 2 {
                    self.simulation_state_seq.states.remove(0);
                }
                self.simulation_state_seq.states.push(simulation_state);
            }

            // for ball in self.simulator.balls.iter() {
            //     println!("{:?} / {:?}", ball.pos.z, self.simulator.world_conf.ball_radius);
            // }
            // println!("");


            if !self.simulation_state_seq.states.is_empty() {
                self.simulation_state = Some(
                    self.simulation_state_seq.calc_interpolated_at(t_elapsed_in_shot));
            }

            // println!("fps: {:?}", timer::get_fps(ctx));

        }

    }

    Ok(())
  }

  fn draw(&mut self, ctx: &mut Context) -> GameResult<()> {
    graphics::clear(ctx);

    if let Some(ref simulation_state) = self.simulation_state {
        for ball in self.simulator.balls.iter() {
            if ball.pos.z < self.graphics_conf.eye_height {
                let distance = (ball.pos.z - self.graphics_conf.eye_height).abs() as f32;
                let scale = self.graphics_conf.pixels_per_meter / distance; 

                graphics::set_color(ctx, graphics::Color::from_rgb(255, 255, 255));
                let ball_graphic = graphics::circle(
                    ctx,
                    graphics::DrawMode::Fill,
                    graphics::Point2::new(
                        self.graphics_conf.origin.x + (ball.pos.x as f32) * scale,
                        self.graphics_conf.origin.y + (ball.pos.y as f32) * scale,
                    ),
                    (self.simulator.world_conf.ball_radius as f32) * scale,
                    0.001,
                );

                // Add some spots on the balls in order to see the rotation.
                // The spot starts on the top of the ball.
                for spot_initial_unit in self.simulator.world_conf.ball_spot_poss.iter() {
                    let spot_initial = spot_initial_unit.unwrap() * self.simulator.world_conf.ball_radius;
                    let spot_rotated =
                            ball.rot.quaternion()
                        *   JQuaternion::new(0., spot_initial.x, spot_initial.y, spot_initial.z)
                        *   ball.rot.inverse().quaternion();
                    let spot_as_vector4 = spot_rotated.as_vector();
                    let spot_as_vector = JVector3::new(spot_as_vector4.x, spot_as_vector4.y, spot_as_vector4.z);
                    if spot_as_vector.z > 0. {

                        let spot_translated = spot_as_vector + ball.pos;

                        graphics::set_color(ctx, graphics::Color::from_rgb(255, 20, 20));
                        let spot_graphic = graphics::circle(
                            ctx,
                            graphics::DrawMode::Fill,
                            graphics::Point2::new(
                                self.graphics_conf.origin.x + (spot_translated.x as f32) * scale,
                                self.graphics_conf.origin.y + (spot_translated.y as f32) * scale,
                            ),
                            (self.simulator.world_conf.ball_radius as f32)
                                * (self.simulator.world_conf.ball_spot_radius_factor as f32) * scale,
                            0.001,
                        );

                    }
                }

            }
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
