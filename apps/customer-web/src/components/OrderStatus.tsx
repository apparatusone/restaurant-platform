// TODO: status bar disappears on page refresh

import { MdOutlinePending, MdOutlineShareLocation, MdTimelapse, MdCheckCircle } from "react-icons/md";

const StatusSteps = {
    Pending: 0,      // PENDING/CONFIRMED
    Preparing: 1,    // IN_PROGRESS  
    OutForDelivery: 2, // OUT_FOR_DELIVERY
    Delivered: 3     // COMPLETED
} as const;

const STEPS = [
    { icon: MdOutlinePending, label: "Pending" },
    { icon: MdTimelapse, label: "Preparing" },
    { icon: MdOutlineShareLocation, label: "Out for Delivery" },
    { icon: MdCheckCircle, label: "Delivered" }
];

interface OrderStatusProps {
    status?: number; // index of step
    orderData?: { trackingNumber?: string; orderId?: number };
    onClose?: () => void;
}

function OrderStatus({ status = StatusSteps.Preparing, orderData, onClose }: OrderStatusProps) {
    const maxIndex = STEPS.length - 1;
    const safeStatus = Math.min(Math.max(status, 0), maxIndex);
    const isCompleted = safeStatus === StatusSteps.Delivered;
    const iconColor = isCompleted ? "text-green-500" : "text-blue-500";
    const progressColor = isCompleted ? "bg-green-500" : "bg-blue-500";
    const progressWidth = `${(safeStatus / maxIndex) * 100}%`;

    return (
        <div className="w-full max-w-md mx-auto pb-4 bg-blue-50 border border-blue-200 rounded-lg relative">
            {onClose && (
                <button
                    onClick={onClose}
                    className="absolute right-2 text-gray-400 hover:text-gray-600 text-xl"
                    title="Close order status"
                >
                    ×
                </button>
            )}

            {orderData && (
                <div className="text-center mb-10">
                    <p className="text-sm font-medium text-gray-700">
                        Order #{orderData.orderId} • {orderData.trackingNumber}
                    </p>
                </div>
            )}

            <div className="relative">
                <div className="h-1 bg-gray-200 rounded-full mx-10">
                    <div
                        className={`h-1 ${progressColor} rounded-full transition-all duration-500 ease-in-out`}
                        style={{ width: progressWidth }}
                    />
                </div>

                <div className="flex justify-between absolute -top-7.5 w-full">
                    {STEPS.map((step, index) => {
                        const Icon = step.icon;
                        const isActive = index <= safeStatus;
                        return (
                            <div className="w-24 flex flex-col items-center" key={step.label}>
                                <span className={`text-xs font-medium ${isActive ? iconColor : "text-gray-400"}`}>{step.label}</span>
                                <div className="relative flex items-center justify-center w-8 h-8">
                                    <div className="absolute rounded-full bg-[#EFF6FF] w-6 h-6" />
                                    <Icon size={24} className={`relative z-10 transition-colors duration-300 ${isActive ? iconColor : "text-gray-400"}`} />
                                </div>
                            </div>
                        );
                    })}
                </div>
            </div>
        </div>
    );
}

export default OrderStatus;