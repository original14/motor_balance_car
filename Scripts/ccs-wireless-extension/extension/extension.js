'use strict';

const fs = require('fs');
const path = require('path');
const childProcess = require('child_process');
const vscode = require('vscode');

let outputChannel;
let activeProcess;

const SCRIPT_BY_COMMAND = Object.freeze({
    'motorBalanceCar.wirelessProbeTest': {
        file: 'wireless_probe_test.bat',
        terminal: 'Wireless Probe Test'
    },
    'motorBalanceCar.wirelessFlash': {
        file: 'wireless_flash.bat',
        terminal: 'Wireless Flash'
    },
    'motorBalanceCar.buildAndWirelessFlash': {
        file: 'build_and_wireless_flash.bat',
        terminal: 'Build + Wireless Flash'
    }
});

function containsWirelessScripts(candidate) {
    return Boolean(candidate) && fs.existsSync(path.join(candidate, 'Scripts', 'build_and_wireless_flash.bat'));
}

function findProjectRoot() {
    const configured = vscode.workspace
        .getConfiguration('motorBalanceCar')
        .get('projectPath', '');

    if (containsWirelessScripts(configured)) {
        return path.resolve(configured);
    }

    for (const folder of vscode.workspace.workspaceFolders || []) {
        if (containsWirelessScripts(folder.uri.fsPath)) {
            return folder.uri.fsPath;
        }

        const childProject = path.join(folder.uri.fsPath, 'motor_balance_car');
        if (containsWirelessScripts(childProject)) {
            return childProject;
        }
    }

    return undefined;
}

function runBatchFile(commandId) {
    const definition = SCRIPT_BY_COMMAND[commandId];
    const projectRoot = findProjectRoot();

    if (!projectRoot) {
        vscode.window.showErrorMessage(
            '找不到 motor_balance_car 工程。请检查设置 motorBalanceCar.projectPath。'
        );
        return;
    }

    const scriptPath = path.join(projectRoot, 'Scripts', definition.file);
    if (!fs.existsSync(scriptPath)) {
        vscode.window.showErrorMessage(`找不到脚本：${scriptPath}`);
        return;
    }

    if (activeProcess && activeProcess.exitCode === null) {
        outputChannel.show(true);
        vscode.window.showWarningMessage('已有 Wireless 命令正在运行，请等待它结束。');
        return;
    }

    const commandShell = process.env.ComSpec || 'C:\\Windows\\System32\\cmd.exe';
    const commandLine = `chcp 65001>nul & call "${scriptPath}"`;
    const startedAt = new Date().toLocaleString();

    outputChannel.appendLine('');
    outputChannel.appendLine(`========== ${definition.terminal} | ${startedAt} ==========`);
    outputChannel.appendLine(`[Extension] Project: ${projectRoot}`);
    outputChannel.appendLine(`[Extension] Command: "${commandShell}" /d /c ${commandLine}`);
    outputChannel.show(true);

    try {
        activeProcess = childProcess.spawn(
            commandShell,
            ['/d', '/c', commandLine],
            {
                cwd: projectRoot,
                env: process.env,
                windowsHide: true,
                windowsVerbatimArguments: true
            }
        );
    } catch (error) {
        outputChannel.appendLine(`[Extension ERROR] ${error.message}`);
        vscode.window.showErrorMessage(`无法启动 Wireless 命令：${error.message}`);
        activeProcess = undefined;
        return;
    }

    activeProcess.stdout.setEncoding('utf8');
    activeProcess.stderr.setEncoding('utf8');
    activeProcess.stdout.on('data', data => outputChannel.append(data));
    activeProcess.stderr.on('data', data => outputChannel.append(data));
    activeProcess.on('error', error => {
        outputChannel.appendLine(`[Extension ERROR] ${error.message}`);
        vscode.window.showErrorMessage(`Wireless 命令启动失败：${error.message}`);
    });
    activeProcess.on('close', code => {
        const exitCode = typeof code === 'number' ? code : -1;
        outputChannel.appendLine('');
        outputChannel.appendLine(`[Extension] Process exited with code ${exitCode}.`);
        if (exitCode === 0) {
            vscode.window.showInformationMessage(`${definition.terminal} 已完成。`);
        } else {
            vscode.window.showErrorMessage(`${definition.terminal} 失败，退出码 ${exitCode}。请查看 Wireless Flash 输出。`);
        }
        activeProcess = undefined;
    });
}

function activate(context) {
    outputChannel = vscode.window.createOutputChannel('Wireless Flash');
    context.subscriptions.push(outputChannel);

    for (const commandId of Object.keys(SCRIPT_BY_COMMAND)) {
        context.subscriptions.push(
            vscode.commands.registerCommand(commandId, () => runBatchFile(commandId))
        );
    }
}

function deactivate() {}

module.exports = { activate, deactivate };
