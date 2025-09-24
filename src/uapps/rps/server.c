#include "compiler.h"
#include "syscall.h"
#include "name.h"
#include "rps/rps.h"
#include "io.h"

#define MAX_GAMES 10
#define MAX_CLIENTS 20

typedef struct {
	int player1_tid;
	int player2_tid;
	rps_choice_t player1_choice;
	rps_choice_t player2_choice;
	int player1_played;
	int player2_played;
	int active;
} game_state_t;

typedef struct {
	game_state_t games[MAX_GAMES];
	int signup_queue[MAX_CLIENTS];
	int queue_head;
	int queue_tail;
	int queue_count;
} server_state_t;

static void init_server_state(server_state_t *state)
{
	for (int i = 0; i < MAX_GAMES; i++) {
		state->games[i].active = 0;
	}
	state->queue_head = 0;
	state->queue_tail = 0;
	state->queue_count = 0;
}

static int enqueue_client(server_state_t *state, int tid)
{
	if (state->queue_count >= MAX_CLIENTS) {
		return -1;
	}

	state->signup_queue[state->queue_tail] = tid;
	state->queue_tail = (state->queue_tail + 1) % MAX_CLIENTS;
	state->queue_count++;
	return 0;
}

static int dequeue_client(server_state_t *state)
{
	if (state->queue_count == 0) {
		return -1;
	}

	int tid = state->signup_queue[state->queue_head];
	state->queue_head = (state->queue_head + 1) % MAX_CLIENTS;
	state->queue_count--;
	return tid;
}

static int find_empty_game(server_state_t *state)
{
	for (int i = 0; i < MAX_GAMES; i++) {
		if (!state->games[i].active) {
			return i;
		}
	}
	return -1;
}

static int find_game_by_tid(server_state_t *state, int tid)
{
	for (int i = 0; i < MAX_GAMES; i++) {
		if (state->games[i].active &&
		    (state->games[i].player1_tid == tid || state->games[i].player2_tid == tid)) {
			return i;
		}
	}
	return -1;
}

static rps_result_t determine_result(rps_choice_t choice1, rps_choice_t choice2)
{
	if (choice1 == choice2) {
		return RPS_TIE;
	}

	if ((choice1 == RPS_ROCK && choice2 == RPS_SCISSORS) || (choice1 == RPS_PAPER && choice2 == RPS_ROCK) ||
	    (choice1 == RPS_SCISSORS && choice2 == RPS_PAPER)) {
		return RPS_WIN;
	}

	return RPS_LOSE;
}

static void handle_signup(server_state_t *state, int client_tid)
{
	console_printf("RPS Server: Received signup from tid %d\r\n", client_tid);

	if (enqueue_client(state, client_tid) < 0) {
		console_printf("RPS Server: Signup queue full, rejecting tid %d\r\n", client_tid);
		rps_response_t response = { .result = -1 };
		Reply(client_tid, (char *)&response, sizeof(response));
		return;
	}

	console_printf("RPS Server: Added tid %d to signup queue (count: %d)\r\n", client_tid, state->queue_count);

	if (state->queue_count >= 2) {
		int game_idx = find_empty_game(state);
		if (game_idx >= 0) {
			int player1 = dequeue_client(state);
			int player2 = dequeue_client(state);

			state->games[game_idx].active = 1;
			state->games[game_idx].player1_tid = player1;
			state->games[game_idx].player2_tid = player2;
			state->games[game_idx].player1_played = 0;
			state->games[game_idx].player2_played = 0;

			console_printf("RPS Server: Starting game %d between tid %d and tid %d\r\n", game_idx, player1,
				       player2);

			rps_response_t response = { .result = 0 };
			Reply(player1, (char *)&response, sizeof(response));
			Reply(player2, (char *)&response, sizeof(response));
		} else {
			console_printf("RPS Server: No available game slots\r\n");
			rps_response_t response = { .result = 0 };
			Reply(client_tid, (char *)&response, sizeof(response));
		}
	} else {
		// Not enough clients to start a game, let the client wait by not replying
		return;
	}
}

static void handle_play(server_state_t *state, int client_tid, rps_choice_t choice)
{
	console_printf("RPS Server: Received play %s from tid %d\r\n", rps_choice_to_string(choice), client_tid);

	int game_idx = find_game_by_tid(state, client_tid);
	if (game_idx < 0) {
		console_printf("RPS Server: Player tid %d not in any active game\r\n", client_tid);
		rps_response_t response = { .result = -1 };
		Reply(client_tid, (char *)&response, sizeof(response));
		return;
	}

	game_state_t *game = &state->games[game_idx];

	if (game->player1_tid == client_tid) {
		game->player1_choice = choice;
		game->player1_played = 1;
	} else {
		game->player2_choice = choice;
		game->player2_played = 1;
	}

	if (game->player1_played && game->player2_played) {
		console_printf("RPS Server: Both players in game %d have played - %s vs %s\r\n", game_idx,
			       rps_choice_to_string(game->player1_choice), rps_choice_to_string(game->player2_choice));

		rps_result_t result1 = determine_result(game->player1_choice, game->player2_choice);
		rps_result_t result2 = determine_result(game->player2_choice, game->player1_choice);

		console_printf("RPS Server: Game %d result - Player 1 (tid %d): %s, Player 2 (tid %d): %s\r\n",
			       game_idx, game->player1_tid, rps_result_to_string(result1), game->player2_tid,
			       rps_result_to_string(result2));

		rps_response_t response1 = { .result = 0,
					     .game_result = result1,
					     .opponent_choice = game->player2_choice };
		rps_response_t response2 = { .result = 0,
					     .game_result = result2,
					     .opponent_choice = game->player1_choice };

		Reply(game->player1_tid, (char *)&response1, sizeof(response1));
		Reply(game->player2_tid, (char *)&response2, sizeof(response2));

		game->player1_played = 0;
		game->player2_played = 0;
	} else {
		// Not the player's turn, let the client wait by not replying
		return;
	}
}

static void handle_quit(server_state_t *state, int client_tid)
{
	console_printf("RPS Server: Received quit from tid %d\r\n", client_tid);

	int game_idx = find_game_by_tid(state, client_tid);
	if (game_idx >= 0) {
		game_state_t *game = &state->games[game_idx];
		int other_player = (game->player1_tid == client_tid) ? game->player2_tid : game->player1_tid;

		console_printf("RPS Server: Player tid %d quit from game %d, notifying opponent tid %d\r\n", client_tid,
			       game_idx, other_player);

		rps_response_t opponent_response = { .result = 0, .game_result = RPS_OPPONENT_QUIT };
		Reply(other_player, (char *)&opponent_response, sizeof(opponent_response));

		game->active = 0;
	}

	rps_response_t response = { .result = 0 };
	Reply(client_tid, (char *)&response, sizeof(response));
}

void __noreturn rps_server_main()
{
	console_printf("RPS Server: Starting up (tid %d)\r\n", MyTid());

	if (RegisterAs(RPS_SERVER_NAME) < 0) {
		console_printf("RPS Server: Failed to register with name server\r\n");
		Exit();
	}

	server_state_t state;
	init_server_state(&state);

	while (1) {
		int sender_tid;
		rps_request_t request;

		int msglen = Receive(&sender_tid, (char *)&request, sizeof(request));
		if (msglen < 0) {
			console_printf("RPS Server: Receive error: %d\r\n", msglen);
			continue;
		}

		if (msglen < (int)sizeof(request)) {
			console_printf("RPS Server: Received truncated message from tid %d\r\n", sender_tid);
			rps_response_t response = { .result = -1 };
			Reply(sender_tid, (char *)&response, sizeof(response));
			continue;
		}

		switch (request.type) {
		case RPS_SIGNUP:
			handle_signup(&state, sender_tid);
			break;
		case RPS_PLAY:
			handle_play(&state, sender_tid, request.choice);
			break;
		case RPS_QUIT:
			handle_quit(&state, sender_tid);
			break;
		default:
			console_printf("RPS Server: Unknown request type %d from tid %d\r\n", request.type, sender_tid);
			rps_response_t response = { .result = -1 };
			Reply(sender_tid, (char *)&response, sizeof(response));
			break;
		}
	}
}
