import cma
from cma.fitness_transformations import EvalParallel2


def cma_par_optimization(loss_func, init_pars, bounds,
                         save_dir="outcmaes/",
                         sigma0=0.4,
                         popsize_factor=3,
                         maxfevals=10000,
                         tolfun=1e-11,
                         tolx=1e-11,
                         tolstagnation=100,
                         num_proc=0):
    if save_dir[-1] != "/":
        sd = save_dir + "/"
    else:
        sd = save_dir
    es = cma.CMAEvolutionStrategy(init_pars,
                                  sigma0,
                                  {'bounds': bounds,
                                   'verbose': -3,
                                   'popsize_factor': popsize_factor,
                                   'verb_filenameprefix': sd,
                                   'maxfevals': maxfevals,
                                   'tolfun': tolfun,
                                   'tolx': tolx,
                                   'tolstagnation': tolstagnation})

    if num_proc > 1:
        with EvalParallel2(loss_func, num_proc) as eval_all:
            while not es.stop():
                X = es.ask()
                es.tell(X, eval_all(X))
                es.disp()
                es.logger.add()  # doctest:+ELLIPSIS
    else:
        es.optimize(loss_func)

    return es.result.xbest
