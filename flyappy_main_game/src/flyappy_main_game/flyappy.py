import os
import random
import sys
from collections.abc import Mapping, Sequence
from itertools import cycle
from pathlib import Path
from typing import Any

import numpy as np
import rclpy
from geometry_msgs.msg import Vector3
from numpy import typing as npt
from rclpy.node import Node
from std_msgs.msg import Bool

os.environ["PYGAME_HIDE_SUPPORT_PROMPT"] = "1"

import pygame
from pygame.locals import K_DOWN, K_ESCAPE, K_LEFT, K_RIGHT, K_SPACE, K_UP, KEYDOWN, QUIT
import pygame.surfarray as surfarray

from .laser import Laser

DEBUG = 0
FPS = 30
SCREENWIDTH = 432
SCREENHEIGHT = 512

# laser specs
LASERFOV = 90.0
LASERRES = 9

# scale pixels to meters
SCALING = 0.01
ACCXLIMIT = 3.0
ACCYLIMIT = 35.0
VELLIMIT = 10.0 / (SCALING * FPS)

PIPESPACING = 192
# amount by which base can maximum shift to left
PIPEGAPSIZE = 50  # gap between upper and lower part of pipe
BASEY = SCREENHEIGHT * 0.79
# image, sound and hitmask  dicts
IMAGES: dict[str, Any] = {}
SOUNDS: dict[str, Any] = {}
HITMASKS: dict[str, Any] = {}

PATHTOFLYAPPY = Path(__file__).parent

# list of all possible players (tuple of 3 positions of flap)
PLAYERS_LIST = (
    # red bird
    (
        PATHTOFLYAPPY / "assets/sprites/redbird-upflap.png",
        PATHTOFLYAPPY / "assets/sprites/redbird-midflap.png",
        PATHTOFLYAPPY / "assets/sprites/redbird-downflap.png",
    ),
)

# list of backgrounds
BACKGROUNDS_LIST = (PATHTOFLYAPPY / "assets/sprites/background-night.png",)

# list of pipes
PIPES_LIST = (PATHTOFLYAPPY / "assets/sprites/pipe-red.png",)

SCREEN: pygame.Surface
FPSCLOCK: pygame.time.Clock


player_acc_x = 0.0
player_acc_y = 0.0


def round_tuple(t: tuple[float, ...]) -> tuple[int, ...]:
    return tuple(round(x) for x in t)


def main() -> None:
    # init ros node
    rclpy.init()
    ros_node = Node("main_game")
    # define subscribers
    ros_node.create_subscription(Vector3, "flyappy_acc", control_callback, 1)

    global SCREEN, FPSCLOCK
    pygame.init()
    FPSCLOCK = pygame.time.Clock()
    SCREEN = pygame.display.set_mode((SCREENWIDTH, SCREENHEIGHT))
    pygame.display.set_caption("Flyappy Game")

    # numbers sprites for score display
    IMAGES["numbers"] = (
        pygame.image.load(PATHTOFLYAPPY / "assets/sprites/0.png").convert_alpha(),
        pygame.image.load(PATHTOFLYAPPY / "assets/sprites/1.png").convert_alpha(),
        pygame.image.load(PATHTOFLYAPPY / "assets/sprites/2.png").convert_alpha(),
        pygame.image.load(PATHTOFLYAPPY / "assets/sprites/3.png").convert_alpha(),
        pygame.image.load(PATHTOFLYAPPY / "assets/sprites/4.png").convert_alpha(),
        pygame.image.load(PATHTOFLYAPPY / "assets/sprites/5.png").convert_alpha(),
        pygame.image.load(PATHTOFLYAPPY / "assets/sprites/6.png").convert_alpha(),
        pygame.image.load(PATHTOFLYAPPY / "assets/sprites/7.png").convert_alpha(),
        pygame.image.load(PATHTOFLYAPPY / "assets/sprites/8.png").convert_alpha(),
        pygame.image.load(PATHTOFLYAPPY / "assets/sprites/9.png").convert_alpha(),
    )

    # game over sprite
    IMAGES["gameover"] = pygame.image.load(
        PATHTOFLYAPPY / "assets/sprites/gameover.png"
    ).convert_alpha()
    # message sprite for welcome screen
    IMAGES["message"] = pygame.image.load(
        PATHTOFLYAPPY / "assets/sprites/message.png"
    ).convert_alpha()
    # base (ground) sprite
    IMAGES["base"] = pygame.image.load(
        PATHTOFLYAPPY / "assets/sprites/base.png"
    ).convert_alpha()

    # sounds
    if "win" in sys.platform:
        sound_ext = ".wav"
    else:
        sound_ext = ".ogg"

    SOUNDS["die"] = pygame.mixer.Sound(PATHTOFLYAPPY / f"assets/audio/die{sound_ext}")
    SOUNDS["hit"] = pygame.mixer.Sound(PATHTOFLYAPPY / f"assets/audio/hit{sound_ext}")
    SOUNDS["point"] = pygame.mixer.Sound(PATHTOFLYAPPY / f"assets/audio/point{sound_ext}")
    SOUNDS["swoosh"] = pygame.mixer.Sound(
        PATHTOFLYAPPY / f"assets/audio/swoosh{sound_ext}"
    )
    SOUNDS["wing"] = pygame.mixer.Sound(PATHTOFLYAPPY / f"assets/audio/wing{sound_ext}")

    try:
        while True:
            # select random background sprites
            rand_bg = 0
            IMAGES["background"] = pygame.image.load(BACKGROUNDS_LIST[rand_bg]).convert()

            # select random player sprites
            rand_player = 0
            IMAGES["player"] = (
                pygame.image.load(PLAYERS_LIST[rand_player][0]).convert_alpha(),
                pygame.image.load(PLAYERS_LIST[rand_player][1]).convert_alpha(),
                pygame.image.load(PLAYERS_LIST[rand_player][2]).convert_alpha(),
            )

            # select random pipe sprites
            pipeindex = 0
            IMAGES["pipe"] = (
                pygame.transform.rotate(
                    pygame.image.load(PIPES_LIST[pipeindex]).convert_alpha(), 180
                ),
                pygame.image.load(PIPES_LIST[pipeindex]).convert_alpha(),
            )

            # hismask for pipes
            HITMASKS["pipe"] = (
                get_hitmask(IMAGES["pipe"][0]),
                get_hitmask(IMAGES["pipe"][1]),
            )

            # hitmask for player
            HITMASKS["player"] = (
                get_hitmask(IMAGES["player"][0]),
                get_hitmask(IMAGES["player"][1]),
                get_hitmask(IMAGES["player"][2]),
            )

            movement_info = show_welcome_animation()
            crash_info = main_game(movement_info, ros_node)
            show_game_over_screen(crash_info)
    except KeyboardInterrupt:
        print("")  # Clean exit
        pygame.quit()


def show_welcome_animation() -> Mapping[str, Any]:
    """Shows welcome screen animation of flyappy game"""
    # index of player to blit on screen
    player_index = 0
    player_index_gen = cycle([0, 1, 2, 1])
    # iterator used to change playerIndex after every 5th iteration
    loop_iter = 0

    playerx = int(SCREENWIDTH * 0.13)
    playery = int((SCREENHEIGHT - IMAGES["player"][0].get_height()) / 2)

    messagex = int((SCREENWIDTH - IMAGES["message"].get_width()) / 2)
    messagey = int(SCREENHEIGHT * 0.12)

    basex = 0
    # amount by which base can maximum shift to left
    base_shift = IMAGES["base"].get_width() - IMAGES["background"].get_width()

    # player shm for up-down motion on welcome screen
    player_shm_vals = {"val": 0, "dir": 1}

    while True:
        for event in pygame.event.get():
            if event.type == QUIT or (event.type == KEYDOWN and event.key == K_ESCAPE):
                pygame.quit()
                sys.exit()
            if event.type == KEYDOWN and (
                event.key == K_UP
                or event.key == K_DOWN
                or event.key == K_LEFT
                or event.key == K_RIGHT
            ):
                # make first flap sound and return values for mainGame
                # SOUNDS['wing'].play()
                return {
                    "playery": playery + player_shm_vals["val"],
                    "basex": basex,
                    "playerIndexGen": player_index_gen,
                }

        # adjust playery, playerIndex, basex
        if (loop_iter + 1) % 5 == 0:
            player_index = next(player_index_gen)
        loop_iter = (loop_iter + 1) % 30
        basex = -((-basex) % base_shift)
        player_shm(player_shm_vals)

        # draw sprites
        SCREEN.blit(IMAGES["background"], (0, 0))
        SCREEN.blit(
            IMAGES["player"][player_index], (playerx, playery + player_shm_vals["val"])
        )
        SCREEN.blit(IMAGES["message"], (messagex, messagey))
        SCREEN.blit(IMAGES["base"], round_tuple((basex, BASEY)))

        pygame.display.update()
        FPSCLOCK.tick(FPS)


def main_game(  # noqa: C901
    movement_info: Mapping[str, Any], ros_node: Node
) -> Mapping[str, Any]:
    global player_acc_x
    global player_acc_y
    # define publishers
    pub_velocity = ros_node.create_publisher(Vector3, "flyappy_vel", 10)
    pub_game_ended = ros_node.create_publisher(Bool, "flyappy_game_ended", 2)
    # create laser
    laser = Laser(LASERFOV, LASERRES, SCALING, ros_node, SCREENWIDTH, SCREENHEIGHT)
    score = player_index = loop_iter = 0
    player_index_gen = movement_info["playerIndexGen"]
    playerx, playery = int(SCREENWIDTH * 0.13), movement_info["playery"]

    # create timer and counter for timer countdown
    pygame.time.set_timer(pygame.USEREVENT, 1000)
    countdown = 60

    #
    basex = movement_info["basex"]
    base_shift = IMAGES["base"].get_width() - IMAGES["background"].get_width()

    # get 2 new pipes to add to upperPipes lowerPipes list
    new_pipe1 = get_random_pipe()
    new_pipe2 = get_random_pipe()

    # list of upper pipes
    upper_pipes = [
        {"x": SCREENWIDTH + 200, "y": new_pipe1[0]["y"]},
        {"x": SCREENWIDTH + 200 + PIPESPACING, "y": new_pipe2[0]["y"]},
    ]

    # list of lowerpipe
    lower_pipes = [
        {"x": SCREENWIDTH + 200, "y": new_pipe1[1]["y"]},
        {"x": SCREENWIDTH + 200 + PIPESPACING, "y": new_pipe2[1]["y"]},
    ]

    # player velocity, max velocity, downward accleration, accleration on flap
    pipe_vel_x = 0.0
    player_vel_y = 0.0  # player's velocity along Y
    delta_vel = 0.8
    between_pipes = 0

    while True:
        # check for crash here
        crash_test = check_crash(
            {"x": playerx, "y": playery, "index": player_index}, upper_pipes, lower_pipes
        )
        if crash_test[0]:
            pub_game_ended.publish(Bool(data=True))
            return {
                "y": playery,
                "groundCrash": crash_test[1],
                "basex": basex,
                "upperPipes": upper_pipes,
                "lowerPipes": lower_pipes,
                "score": score,
                "playerVelY": player_vel_y,
                "timeRanOut": 0,
            }

        for event in pygame.event.get():
            if event.type == QUIT or (event.type == KEYDOWN and event.key == K_ESCAPE):
                pygame.quit()
                sys.exit()
            if event.type == KEYDOWN and (event.key == K_UP):
                # if playery > -2 * IMAGES['player'][0].get_height():
                player_vel_y -= delta_vel
                # playerAccY -= deltaAcc
            if event.type == KEYDOWN and (event.key == K_DOWN):
                player_vel_y += delta_vel
                # playerAccY += deltaAcc
            if event.type == KEYDOWN and (event.key == K_LEFT):
                pipe_vel_x += delta_vel
                # playerAccX += deltaAcc
            if event.type == KEYDOWN and (event.key == K_RIGHT):
                pipe_vel_x -= delta_vel
                # playerAccX -= deltaAcc
            if event.type == pygame.USEREVENT and score > 0:
                if countdown > 0:
                    countdown -= 1
                else:
                    pub_game_ended.publish(Bool(data=False))
                    return {
                        "y": playery,
                        "groundCrash": crash_test[1],
                        "basex": basex,
                        "upperPipes": upper_pipes,
                        "lowerPipes": lower_pipes,
                        "score": score,
                        "playerVelY": player_vel_y,
                        "timeRanOut": 1,
                    }

        # update velocity
        rclpy.spin_once(ros_node, timeout_sec=0)
        pipe_vel_x += player_acc_x
        player_vel_y += player_acc_y

        # limit velocity
        player_vel_y = limit_vel(player_vel_y, 1)
        pipe_vel_x = limit_vel(pipe_vel_x, 0)

        # publish pub_velocity
        pub_velocity.publish(
            Vector3(
                x=float(-SCALING * FPS * pipe_vel_x),
                y=float(-SCALING * FPS * player_vel_y),
                z=float(0.0),
            )
        )

        # check for score
        player_mid_pos = playerx + IMAGES["player"][0].get_width() / 2
        pipe_counter = 0
        for pipe in upper_pipes:
            pipe_mid_pos = pipe["x"] + IMAGES["pipe"][0].get_width() / 2
            if (
                pipe_mid_pos
                <= player_mid_pos
                < (pipe_mid_pos + IMAGES["pipe"][0].get_width() / 2)
            ):
                pipe_counter += 1
                if between_pipes == 0:
                    score += 1
                    between_pipes = 1
                    SOUNDS["point"].play()
        if pipe_counter == 0:
            between_pipes = 0
        # playerIndex basex change
        if (loop_iter + 1) % 3 == 0:
            player_index = next(player_index_gen)
        loop_iter = (loop_iter + 1) % 30
        basex = -((-basex - pipe_vel_x) % base_shift)

        player_height = IMAGES["player"][player_index].get_height()
        playery += min(player_vel_y, BASEY - playery - player_height)

        # move pipes to left
        for u_pipe, l_pipe in zip(upper_pipes, lower_pipes):
            u_pipe["x"] += pipe_vel_x
            l_pipe["x"] += pipe_vel_x

        # add new pipe when first pipe each ? pixels
        if (SCREENWIDTH + 200) > upper_pipes[-1]["x"]:
            new_pipe = get_random_pipe()
            new_pipe[0]["x"] = PIPESPACING + upper_pipes[-1]["x"]
            new_pipe[1]["x"] = PIPESPACING + upper_pipes[-1]["x"]
            upper_pipes.append(new_pipe[0])
            lower_pipes.append(new_pipe[1])

        # remove first pipe if its out of the screen
        if upper_pipes[0]["x"] < -IMAGES["pipe"][0].get_width():
            upper_pipes.pop(0)
            lower_pipes.pop(0)

        # draw sprites
        SCREEN.blit(IMAGES["background"], (0, 0))

        for u_pipe, l_pipe in zip(upper_pipes, lower_pipes):
            SCREEN.blit(IMAGES["pipe"][0], round_tuple((u_pipe["x"], u_pipe["y"])))
            SCREEN.blit(IMAGES["pipe"][1], round_tuple((l_pipe["x"], l_pipe["y"])))

        SCREEN.blit(IMAGES["base"], round_tuple((basex, BASEY)))
        ###################################################################
        # get bitmap of obstacles
        bitmap = get_bitmap(upper_pipes, lower_pipes, round_tuple((basex, BASEY)))
        # do raytracing with Laser
        player_middle = (
            playerx + IMAGES["player"][0].get_width() / 2,
            playery + IMAGES["player"][0].get_height() / 2,
        )
        laser_points = laser.scan(player_middle, bitmap)

        # display
        if DEBUG == 1:
            # display obstacles and ray tracing
            bitmap_surf = pygame.surfarray.make_surface(bitmap)
            SCREEN.blit(bitmap_surf, (0, 0))

        for i in range(len(laser_points)):
            if laser_points[i][2] == 1:
                pygame.draw.circle(SCREEN, (0, 255, 0), laser_points[i][0:2], 3, 0)
                pygame.draw.aaline(
                    SCREEN, (0, 255, 0), player_middle, laser_points[i][0:2], 1
                )
            else:
                pygame.draw.aaline(
                    SCREEN, (0, 140, 0), player_middle, laser_points[i][0:2], 1
                )
        ###################################################################

        # print score so player overlaps the score
        show_score(score)
        if score > 0:
            show_counter(countdown)
        player_surface = pygame.transform.rotate(IMAGES["player"][player_index], 0)
        SCREEN.blit(player_surface, round_tuple((playerx, playery)))

        pygame.display.update()
        FPSCLOCK.tick(FPS)


def show_game_over_screen(crash_info: Mapping[str, Any]) -> None:  # noqa: C901
    """crashes the player down ans shows gameover image"""
    score = crash_info["score"]
    playerx = SCREENWIDTH * 0.13
    playery = crash_info["y"]
    player_height = IMAGES["player"][0].get_height()
    player_vel_y = crash_info["playerVelY"]
    player_acc_y = 2

    basex = crash_info["basex"]

    upper_pipes, lower_pipes = crash_info["upperPipes"], crash_info["lowerPipes"]

    # play hit and die sounds
    if not crash_info["timeRanOut"]:
        SOUNDS["hit"].play()
    if not crash_info["groundCrash"]:
        SOUNDS["die"].play()

    while True:
        for event in pygame.event.get():
            if event.type == QUIT or (event.type == KEYDOWN and event.key == K_ESCAPE):
                pygame.quit()
                sys.exit()
            if event.type == KEYDOWN and (event.key == K_SPACE or event.key == K_UP):
                if playery + player_height >= BASEY - 1:
                    return

        # player y shift
        if playery + player_height < BASEY - 1:
            playery += min(player_vel_y, BASEY - playery - player_height)

        # player velocity change
        if player_vel_y < 15:
            player_vel_y += player_acc_y

        # draw sprites
        SCREEN.blit(IMAGES["background"], (0, 0))

        for u_pipe, l_pipe in zip(upper_pipes, lower_pipes):
            SCREEN.blit(IMAGES["pipe"][0], round_tuple((u_pipe["x"], u_pipe["y"])))
            SCREEN.blit(IMAGES["pipe"][1], round_tuple((l_pipe["x"], l_pipe["y"])))

        SCREEN.blit(IMAGES["base"], round_tuple((basex, BASEY)))
        show_score(score)

        player_surface = pygame.transform.rotate(IMAGES["player"][1], -45)
        SCREEN.blit(player_surface, round_tuple((playerx, playery)))

        FPSCLOCK.tick(FPS)
        pygame.display.update()


def player_shm(player_shm: dict[str, int]) -> None:
    """oscillates the value of playerShm['val'] between 8 and -8"""
    if abs(player_shm["val"]) == 8:
        player_shm["dir"] *= -1

    if player_shm["dir"] == 1:
        player_shm["val"] += 1
    else:
        player_shm["val"] -= 1


def get_random_pipe() -> Sequence[dict[str, float]]:
    """returns a randomly generated pipe"""
    # y of gap between upper and lower pipe
    gap_y = random.randrange(0, int(BASEY * 0.6 - PIPEGAPSIZE))
    gap_y += int(BASEY * 0.2)
    pipe_height = IMAGES["pipe"][0].get_height()
    pipe_x = SCREENWIDTH + 10

    return [
        {"x": pipe_x, "y": gap_y - pipe_height},  # upper pipe
        {"x": pipe_x, "y": gap_y + PIPEGAPSIZE},  # lower pipe
    ]


def show_score(score: int) -> None:
    """displays score in center of screen"""
    score_digits = [int(x) for x in list(str(score))]
    total_width = 0  # total width of all numbers to be printed

    for digit in score_digits:
        total_width += IMAGES["numbers"][digit].get_width()

    x_offset = (SCREENWIDTH - total_width) / 2

    for digit in score_digits:
        SCREEN.blit(IMAGES["numbers"][digit], round_tuple((x_offset, SCREENHEIGHT * 0.1)))
        x_offset += IMAGES["numbers"][digit].get_width()


def show_counter(counter: int) -> None:
    """displays score in center of screen"""
    score_digits = [int(x) for x in list(str(counter))]
    total_width = 0  # total width of all numbers to be printed

    for digit in score_digits:
        total_width += IMAGES["numbers"][digit].get_width()

    x_offset = (SCREENWIDTH - total_width) / 2

    for digit in score_digits:
        SCREEN.blit(
            IMAGES["numbers"][digit], round_tuple((x_offset, SCREENHEIGHT * 0.85))
        )
        x_offset += IMAGES["numbers"][digit].get_width()


def check_crash(
    player: dict[str, Any],
    upper_pipes: list[dict[str, float]],
    lower_pipes: list[dict[str, float]],
) -> tuple[bool, bool]:
    """returns True if player collders with top, base or pipes."""
    pi = player["index"]
    player["w"] = IMAGES["player"][0].get_width()
    player["h"] = IMAGES["player"][0].get_height()

    # if player crashes into ground
    if player["y"] + player["h"] >= BASEY - 1:
        return (True, True)
    elif player["y"] <= 0:
        return (True, True)
    else:

        player_rect = pygame.Rect(
            *round_tuple((player["x"], player["y"], player["w"], player["h"]))
        )
        pipe_w = IMAGES["pipe"][0].get_width()
        pipe_h = IMAGES["pipe"][0].get_height()

        for u_pipe, l_pipe in zip(upper_pipes, lower_pipes):
            # upper and lower pipe rects
            u_pipe_rect = pygame.Rect(
                *round_tuple((u_pipe["x"], u_pipe["y"], pipe_w, pipe_h))
            )
            l_pipe_rect = pygame.Rect(
                *round_tuple((l_pipe["x"], l_pipe["y"], pipe_w, pipe_h))
            )

            # player and upper/lower pipe hitmasks
            p_hit_mask = HITMASKS["player"][pi]
            u_hit_mask = HITMASKS["pipe"][0]
            l_hit_mask = HITMASKS["pipe"][1]

            # if bird collided with upipe or lpipe
            u_collide = pixel_collision(player_rect, u_pipe_rect, p_hit_mask, u_hit_mask)
            l_collide = pixel_collision(player_rect, l_pipe_rect, p_hit_mask, l_hit_mask)

            if u_collide or l_collide:
                return (True, False)

    return (False, False)


def pixel_collision(
    rect1: pygame.Rect,
    rect2: pygame.Rect,
    hitmask1: Sequence[Sequence[bool]],
    hitmask2: Sequence[Sequence[bool]],
) -> bool:
    """Checks if two objects collide and not just their rects"""
    rect = rect1.clip(rect2)

    if rect.width == 0 or rect.height == 0:
        return False

    x1, y1 = rect.x - rect1.x, rect.y - rect1.y
    x2, y2 = rect.x - rect2.x, rect.y - rect2.y

    for x in range(rect.width):
        for y in range(rect.height):
            if hitmask1[x1 + x][y1 + y] and hitmask2[x2 + x][y2 + y]:
                return True
    return False


def get_hitmask(image: pygame.Surface) -> Sequence[Sequence[bool]]:
    """returns a hitmask using an image's alpha."""
    mask: list[list[bool]] = []
    for x in range(image.get_width()):
        mask.append([])
        for y in range(image.get_height()):
            mask[x].append(bool(image.get_at((x, y))[3]))
    return mask


def get_bitmap(
    upper_pipes: Sequence[Mapping[str, float]],
    lower_pipes: Sequence[Mapping[str, float]],
    base: tuple[int, ...],
) -> npt.NDArray[np.uint8]:
    obstacle_surface = pygame.Surface((SCREENWIDTH, SCREENHEIGHT), flags=pygame.SRCALPHA)
    obstacle_surface.fill((0, 0, 0, 0))
    # Add obstacles
    for u_pipe, l_pipe in zip(upper_pipes, lower_pipes):
        obstacle_surface.blit(IMAGES["pipe"][0], round_tuple((u_pipe["x"], u_pipe["y"])))
        obstacle_surface.blit(IMAGES["pipe"][1], round_tuple((l_pipe["x"], l_pipe["y"])))
        obstacle_surface.blit(IMAGES["base"], base)

    # Copy alpha values
    # bitmap = pygame.mask.from_surface(obstacleSurface,255)
    bitmap = surfarray.array_alpha(obstacle_surface)
    return bitmap


def control_callback(data: Vector3) -> None:
    global player_acc_x
    global player_acc_y

    player_acc_x = limit_acceleration(-data.x, ACCXLIMIT) / (FPS * FPS * SCALING)
    player_acc_y = limit_acceleration(-data.y, ACCYLIMIT) / (FPS * FPS * SCALING)


def limit_acceleration(acc_user: float, limit: float) -> float:
    if acc_user > limit:
        acc_user = limit
    elif acc_user < -limit:
        acc_user = -limit
    return acc_user


def limit_vel(vel_user: float, direction: int) -> float:
    if vel_user > VELLIMIT and direction == 1:
        vel_user = VELLIMIT
    elif vel_user > 0 and direction == 0:
        vel_user = 0
    elif vel_user < -VELLIMIT:
        vel_user = -VELLIMIT
    return vel_user


if __name__ == "__main__":
    main()
