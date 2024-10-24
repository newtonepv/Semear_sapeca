import asyncio

async def pid_loop():
    while True:
        print("PID loop executando...")
        await asyncio.sleep(1)  # Simula trabalho com pausa não bloqueante

async def detect_loop():
    while True:
        print("Detector executando...")
        await asyncio.sleep(1.5)  # Simula trabalho com outra frequência

async def main():
    # Cria tarefas para rodar simultaneamente
    task1 = asyncio.create_task(pid_loop())
    task2 = asyncio.create_task(detect_loop())

    # Mantém as tarefas vivas até serem canceladas
    await asyncio.gather(task1, task2)

# Inicia o loop de eventos
asyncio.run(main())
