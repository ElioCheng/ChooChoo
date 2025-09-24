#ifndef __UAPPS_RPS_H__
#define __UAPPS_RPS_H__

#define RPS_SERVER_NAME "rps_server"

typedef enum { RPS_ROCK = 1, RPS_PAPER = 2, RPS_SCISSORS = 3 } rps_choice_t;

typedef enum { RPS_SIGNUP = 1, RPS_PLAY = 2, RPS_QUIT = 3 } rps_request_type_t;

typedef enum { RPS_WIN = 1, RPS_LOSE = 2, RPS_TIE = 3, RPS_OPPONENT_QUIT = 4 } rps_result_t;

typedef struct {
	rps_request_type_t type;
	rps_choice_t choice;
} rps_request_t;

typedef struct {
	int result;
	rps_result_t game_result;
	rps_choice_t opponent_choice;
} rps_response_t;

static inline const char *__attribute__((optimize("O1"))) rps_choice_to_string(rps_choice_t choice)
{
	switch (choice) {
	case RPS_ROCK:
		return "Rock";
	case RPS_PAPER:
		return "Paper";
	case RPS_SCISSORS:
		return "Scissors";
	default:
		return "Unknown";
	}
}

static inline const char *__attribute__((optimize("O1"))) rps_result_to_string(rps_result_t result)
{
	switch (result) {
	case RPS_WIN:
		return "Win";
	case RPS_LOSE:
		return "Lose";
	case RPS_TIE:
		return "Tie";
	case RPS_OPPONENT_QUIT:
		return "Opponent Quit";
	default:
		return "Unknown";
	}
}

void rps_server_main(void);
void rps_client_main(void);

#endif /* __UAPPS_RPS_H__ */
