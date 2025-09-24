#include "compiler.h"
#include "syscall.h"
#include "name.h"
#include "rps/rps.h"
#include "random.h"
#include "io.h"

typedef struct {
	int server_tid;
	int my_tid;
	int round;
} client_state_t;

static int send_rps_request(client_state_t *state, rps_request_t *request, rps_response_t *response)
{
	int result = Send(state->server_tid, (char *)request, sizeof(*request), (char *)response, sizeof(*response));

	if (result < 0) {
		console_printf("RPS Client %d: Send failed with error %d\r\n", state->my_tid, result);
		return -1;
	}

	if (result < (int)sizeof(*response)) {
		console_printf("RPS Client %d: Received truncated response (%d bytes)\r\n", state->my_tid, result);
		return -1;
	}

	return 0;
}

static int signup(client_state_t *state)
{
	console_printf("RPS Client %d: Sending signup request\r\n", state->my_tid);

	rps_request_t request = { .type = RPS_SIGNUP };
	rps_response_t response;

	if (send_rps_request(state, &request, &response) < 0) {
		return -1;
	}

	console_printf("RPS Client %d: Signup response - result: %d\r\n", state->my_tid, response.result);

	if (response.result < 0) {
		console_printf("RPS Client %d: Signup failed\r\n", state->my_tid);
		return -1;
	}

	console_printf("RPS Client %d: Signup successful, waiting to be paired\r\n", state->my_tid);
	return 0;
}

static int play_round(client_state_t *state, rps_choice_t choice)
{
	console_printf("RPS Client %d: Playing %s (round %d)\r\n", state->my_tid, rps_choice_to_string(choice),
		       state->round + 1);

	rps_request_t request = { .type = RPS_PLAY, .choice = choice };
	rps_response_t response;

	if (send_rps_request(state, &request, &response) < 0) {
		return -1;
	}

	if (response.result < 0) {
		console_printf("RPS Client %d: Play request failed\r\n", state->my_tid);
		return -1;
	}

	if (response.game_result == RPS_OPPONENT_QUIT) {
		console_printf("RPS Client %d: Opponent quit the game\r\n", state->my_tid);
		return -2;
	}

	if (response.game_result != 0) {
		console_printf("RPS Client %d: Round %d Completed - %s vs %s (%s)\r\n", state->my_tid, state->round + 1,
			       rps_choice_to_string(choice), rps_choice_to_string(response.opponent_choice),
			       rps_result_to_string(response.game_result));
		state->round++;
		return 1;
	}

	console_printf("RPS Client %d: Waiting for opponent to play\r\n", state->my_tid);
	return 0;
}

static int quit(client_state_t *state)
{
	console_printf("RPS Client %d: Sending quit request\r\n", state->my_tid);

	rps_request_t request = { .type = RPS_QUIT };
	rps_response_t response;

	if (send_rps_request(state, &request, &response) < 0) {
		return -1;
	}

	console_printf("RPS Client %d: Quit response - result: %d\r\n", state->my_tid, response.result);
	return 0;
}

static void gameplay(client_state_t *state)
{
	if (signup(state) < 0) {
		console_printf("RPS Client %d: Signup failed\r\n", state->my_tid);
		return;
	}

	for (int i = 0; i < 3; i++) {
		rps_choice_t choice = (random() % 3) + 1;
		int result = play_round(state, choice);
		if (result < 0) {
			if (result == -2) {
				console_printf("RPS Client %d: Game ended due to opponent quit\r\n", state->my_tid);
				return;
			}
			console_printf("RPS Client %d: Play failed\r\n", state->my_tid);
			return;
		}

		// for (volatile int j = 0; j < 1000000; j++) {
		// 	// Delay a bit
		// }
	}

	quit(state);
}

void __noreturn rps_client_main()
{
	client_state_t state;
	state.my_tid = MyTid();
	state.round = 0;

	console_printf("RPS Client: Starting up (tid %d)\r\n", state.my_tid);

	state.server_tid = WhoIs(RPS_SERVER_NAME);
	if (state.server_tid < 0) {
		console_printf("RPS Client %d: Failed to find RPS server '%s'\r\n", state.my_tid, RPS_SERVER_NAME);
		Exit();
	}

	console_printf("RPS Client %d: Found RPS server at tid %d\r\n", state.my_tid, state.server_tid);
	gameplay(&state);

	console_printf("RPS Client %d: Exiting\r\n", state.my_tid);
	Exit();
	UNREACHABLE();
}
