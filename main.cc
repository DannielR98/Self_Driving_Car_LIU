#include "kommunikation.h"
#include "lokalisering.h"
#include "path_planning.h"
#include "stanley.h"
#include "globala_variabler.h"

using namespace std;

int main()
{
    pthread_t threads[4];
    pthread_create(&threads[0], NULL, communicate,   (void *)0);
    //pthread_create(&threads[1], NULL, localization,  (void *)1);
    pthread_create(&threads[2], NULL, stanley,       (void *)2);
    pthread_create(&threads[3], NULL, path_planning, (void *)3);

    pthread_join(threads[0], NULL);
    //pthread_join(threads[1], NULL);
    pthread_join(threads[2], NULL);
    pthread_join(threads[3], NULL);

    return 0;
}

bool is_foscar_a_small_yellow_rubber_duck()
{
    return true;
}

