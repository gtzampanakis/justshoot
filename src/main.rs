extern crate ggez;
use ggez::*;

mod jlib;
mod geometry;

use jlib::{
    Ball,
    Simulator,
    WorldConf,
    consts,
};

use geometry::{
    JVector3,
    JUnitQuaternion,
};

struct GameState {
    simulator: Simulator,
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
                u: JVector3::zeros(),
                rot: JUnitQuaternion::identity(),
            },
        ];
        
        GameState {
            simulator: Simulator::new(
                balls,
                world_conf,
                1e-4,
            )
        }
    }
}

impl ggez::event::EventHandler for GameState {
  fn update(&mut self, ctx: &mut Context) -> GameResult<()> {
    Ok(())
  }

  fn draw(&mut self, ctx: &mut Context) -> GameResult<()> {
    Ok(())
  }
}

fn show() {
    let mut game_state = GameState::new();

    let c = conf::Conf::new();
    let ctx = &mut Context::load_from_conf("hello_ggez", "awesome_person", c).unwrap();

    event::run(ctx, &mut game_state).unwrap();
}

fn main() {
    show();
}
